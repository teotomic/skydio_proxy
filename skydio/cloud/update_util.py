"""
Utilities to login to the skydio cloud api and update the skillsets on a vehicle
via cloud config. For use by external parties without any internal skydio dependencies.
"""

import argparse
import base64
import os
import logging
import json
import socket
import sys
from functools import wraps

import requests

LOGGER_NAME = "CloudAPIClient"
CLOUD_API_HOME_DIR = os.path.expanduser("~/.skydio_cloud_api")
CODE_HEADER = "X-Api-Status-Code"
DIRECT_PROTO_RESPONSE_MESSAGE_HEADER = "X-use-direct-message-proto-response"
DEFAULT_CLOUD_URL = "https://api.skydio.com"


class CloudAPIException(Exception):

    def __init__(self, method, route, error=None, code=None):
        super(CloudAPIException, self).__init__()
        self.method = method
        self.route = route
        self.error = error
        self.code = code

    @property
    def message(self):
        return str(self)

    def __str__(self):
        return '{} to {} replied w/ error {}: {}'.format(self.method.upper(), self.route,
                                                         self.code, self.error)


def _refresh_if_needed(func):
    """
    Decorator that calls refresh() and retries request if response indicates expired access token
    """
    @wraps(func)
    def wrap(self, *pargs, **kwargs):
        skip_auto_refresh = kwargs.pop('skip_auto_refresh', False)
        try:
            return func(self, *pargs, **kwargs)
        except CloudAPIException as err:
            if not skip_auto_refresh and err.code == 3100:
                self.log.debug("Refreshing access token...")
                self.refresh()
                return func(self, *pargs, **kwargs)
            raise
    return wrap


def _dirname_for_url(url):
    return base64.urlsafe_b64encode(url.encode('utf-8')).decode('utf-8')


def _store_local_refresh_token(user_email, token, url):
    dirpath = os.path.join(CLOUD_API_HOME_DIR, _dirname_for_url(url))
    if not os.path.isdir(dirpath):
        os.makedirs(dirpath)

    with open(os.path.join(dirpath, user_email), 'w') as refresh_file:
        refresh_file.write(token)


def _local_refresh_token(user_email, url):
    dirpath = os.path.join(CLOUD_API_HOME_DIR, _dirname_for_url(url))

    if os.path.isdir(dirpath):
        candidate = os.path.join(dirpath, user_email)
        if os.path.isfile(candidate):
            with open(candidate, 'r') as refresh_file:
                return refresh_file.read()

    return None


def _remove_local_refresh_token(user_email, url):
    dirpath = os.path.join(CLOUD_API_HOME_DIR, _dirname_for_url(url))

    if os.path.isdir(dirpath):
        candidate = os.path.join(dirpath, user_email)
        if os.path.isfile(candidate):
            os.remove(candidate)


class AttrDict(dict):
    """Simple attr-dict that constructs recursively."""

    def __init__(self, **kwargs):
        modified_args = {k: self.recursive_transform(v) for (k, v) in kwargs.items()}
        super(AttrDict, self).__init__(**modified_args)
        self.__dict__ = self

    @classmethod
    def recursive_transform(cls, item):
        if isinstance(item, dict):
            return AttrDict(**item)
        else:
            return item


class CloudAPIClient(object):
    """
    A base client which sets the correct headers to interact with the API and parses application
    errors appropriately
    """
    BASE_HEADERS = {'Content-Type': 'application/json'}
    LOGGING_CONFIGURED = False

    def __init__(self, url, user_email, access_token=None, refresh_token=None,
                 use_stored_tokens=True):
        self.url = url
        self.log = logging.getLogger(LOGGER_NAME)
        self._use_stored_tokens = use_stored_tokens

        self.user_email = user_email
        self.access_token = access_token
        self.refresh_token = refresh_token
        if not self.refresh_token and self._use_stored_tokens:
            self.refresh_token = _local_refresh_token(user_email, url)

        self.device_id = 'python_client_{}'.format(socket.gethostname())
        self.client_key = "python_client"

    def login_interactive(self):
        """
        Check if we have the required auth tokens, else call login and prompt the user to enter
        the login code from their email
        """
        # If we dont yet have a refresh token we need to ask the user to login for one
        if not self.refresh_token:
            self.log.info("Refresh token needed, calling login endpoint for {}".format(
                self.user_email))
            self.login()
            self.log.info("Please check your email for the login code")
            login_code = raw_input("Code: ")
            self.authenticate(login_code)

        if not self.access_token:
            self.log.info("No access token found, calling refresh")
            self.refresh()

    def login(self):
        """
        Send a login_code to the user_email
        """
        return self.post('auth/login', {"email": self.user_email}, skip_token_check=True)

    def authenticate(self, login_code):
        """
        Use a login code to obtain access token credentials
        """
        reply = self.post('auth/authenticate', {"email": self.user_email,
                                                "login_code": int(login_code),
                                                "device_id": self.device_id,
                                                "client_key": self.client_key},
                          skip_token_check=True)
        self.access_token = reply.access_token
        self.refresh_token = reply.refresh_token

        if self._use_stored_tokens:
            _store_local_refresh_token(self.user_email, self.refresh_token, self.url)

    def refresh(self):
        """
        Obtain a new access token
        """
        assert self.refresh_token is not None
        try:
            reply = self.post('auth/refresh', skip_token_check=True,  # pylint: disable=unexpected-keyword-arg
                              use_refresh_token=True, skip_auto_refresh=True)
            self.access_token = reply.access_token
        except CloudAPIException as err:
            if err.code in (3100, 3300):
                if self._use_stored_tokens:
                    if _local_refresh_token(self.user_email, self.url) == self.refresh_token:
                        # the refresh token expired, delete any cached entry so we re-auth
                        _remove_local_refresh_token(self.user_email, self.url)
            raise

    @classmethod
    def log_to_stdout(cls, debug=True):
        if cls.LOGGING_CONFIGURED:
            return
        logger = logging.getLogger(LOGGER_NAME)
        logger.setLevel(logging.INFO)
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(logging.NOTSET)
        logger.addHandler(handler)
        if debug:
            logger.setLevel(logging.DEBUG)
        logger.propagate = False
        cls.LOGGING_CONFIGURED = True

    def _endpoint(self, route):
        return "{}/{}".format(self.url.rstrip('/'), route.lstrip('/'))

    @_refresh_if_needed
    def post(self, route, data=None, timeout=3, skip_token_check=False, use_refresh_token=False,
             send_proto_data=False):
        headers = dict()
        headers.update(self.BASE_HEADERS)

        if not skip_token_check and not self.access_token:
            raise Exception("Must authenticate before accessing regular API routes")

        if self.access_token or self.refresh_token:
            headers['Authorization'] = "Bearer {}".format(self.access_token if not
                                                          use_refresh_token else self.refresh_token)

        endpoint = self._endpoint(route)
        self.log.debug('POST {}'.format(endpoint))

        # Sometimes we want to send a request with proto content-type so that we get a proto resp
        if send_proto_data:
            headers[DIRECT_PROTO_RESPONSE_MESSAGE_HEADER] = '1'
            headers['Content-Type'] = 'application/x-protobuf'
            data = data
        else:
            headers['Content-Type'] = 'application/json'
            data = json.dumps(data)
        res = requests.post(url=endpoint, data=data, timeout=timeout, headers=headers)
        return self._handle_response('post', route, res)

    @_refresh_if_needed
    def get(self, route, params=None, timeout=3, skip_token_check=False):
        headers = dict()
        headers.update(self.BASE_HEADERS)

        if not skip_token_check and not self.access_token:
            raise Exception("Must authenticate before accessing regular API routes")

        if self.access_token:
            headers['Authorization'] = "Bearer {}".format(self.access_token)

        endpoint = self._endpoint(route)
        self.log.debug('GET {}'.format(endpoint))
        res = requests.get(url=endpoint, params=params, timeout=timeout, headers=headers)
        return self._handle_response('get', route, res)

    def _handle_response(self, method, route, res):
        assert isinstance(res, requests.Response)

        # The API should respond with a 200 status code for all requests and application-level
        # errors will be included in the response

        # If there are any http-level errors, raise them
        try:
            res.raise_for_status()
        except requests.HTTPError as err:
            raise CloudAPIException(method, route, error=err.message, code=res.status_code)

        # Check for any errors
        error_code = int(res.headers[CODE_HEADER])
        if error_code != 0:
            raise CloudAPIException(method, route, code=error_code)

        if res.headers['Content-Type'] == 'application/json':
            try:
                reply = res.json()
            except ValueError as err:
                raise CloudAPIException(method, route,
                                        error="Couldn't decode json: {}".format(err.message))

            # convert to attribute dict
            reply = AttrDict(**reply)

            return reply.data

        elif res.headers['Content-Type'] == 'application/x-protobuf':
            return res.content


def update_cloud_config_on_vehicle(user_email, vehicle_url, vehicle_access_token,
                                   cloud_url=DEFAULT_CLOUD_URL):
    """
    Retrieve the cloud config (which includes skillsets) for the given user_email from the
    Skydio Cloud API and upload to the vehicle.

    If this is the first time running this script on this computer an interactive prompt
    will be shown to get the login_code sent to the user_email. After that it should persist.

    This function is dumber than what the Mobile App would do since it does not cache
    any parts of the cloud config or decode the message such that we can determine if
    it's necessary to send to the vehicle. We just send the full config to the vehicle every time.
    """
    if not cloud_url:
        cloud_url = DEFAULT_CLOUD_URL

    client = CloudAPIClient(url=cloud_url, user_email=user_email)
    client.log_to_stdout(debug=False)

    client.login_interactive()

    # Get the encoded cloud config from the api server
    print("Getting cloud config from API server...")
    cconfig = client.post('cloud_config', '', timeout=5, send_proto_data=True)

    # Send the encoded cloud config to the vehicle
    print("Sending cloud config to the vehicle...")
    res = requests.post(url="{}/api/update_cloud_config".format(vehicle_url),
                        data=cconfig,
                        headers={"Authorization": "Bearer {}".format(vehicle_access_token),
                                 "Accept": "application/json",
                                 "Content-Type": "application/x-protobuf"},
                        timeout=3)
    # Raise an exception if we errored in sending the cloud config
    res.raise_for_status()

    # Tell the vehicle to restart the skills process ... this may error if the skills front
    # is not yet running so we don't check for errors on this request
    print("Restarting skills process...")
    requests.post(url="{}/api/cloud_config_skills_refresh".format(vehicle_url),
                  data=json.dumps({"force": True}),
                  headers={"Authorization": "Bearer {}".format(vehicle_access_token),
                           "Accept": "application/json",
                           "Content-Type": "application/json"},
                  timeout=20)

    print("Successfully synchronized skills")


def main():
    parser = argparse.ArgumentParser(description="Update the skillsets on a vehicle from the cloud")

    parser.add_argument('user_email', type=str, help='The email of the user to get skillsets for')
    parser.add_argument('vehicle_url', type=str, metavar='URL', default='http://192.168.10.1',
                        help='the url of the vehicle')
    parser.add_argument('access_token', type=str, help='A PILOT access_token for the vehicle')
    parser.add_argument('--skydio-api-url', type=str, default=DEFAULT_CLOUD_URL,
                        help='Override the skydio api url')

    args = parser.parse_args()

    update_cloud_config_on_vehicle(args.user_email, args.vehicle_url, args.access_token,
                                   cloud_url=args.skydio_api_url)

if __name__ == '__main__':
    main()
