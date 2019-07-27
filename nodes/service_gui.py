#!/usr/bin/env python
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Pango

import signal
import rospy
from skydio_proxy.msg import SkillStatus
from skydio_proxy.srv import SkydioCommand


class SkydioServiceWindow(Gtk.Window):

    def __init__(self):
        Gtk.Window.__init__(self, title="Mission control")

        service_path = rospy.get_param('~service_path', '/skydio/command')
        self.service = rospy.ServiceProxy(service_path, SkydioCommand)

        self.box = Gtk.Box(spacing=10)
        self.add(self.box)

        commands = (('Prep', 'prep'),
                    ('Take off', 'takeoff'),
                    ('Start mission', 'start_mission'),
                    ('Land', 'land'),
                    ('Sync skills', 'update_skillset'),
                    )

        for label, command in commands:
            button = Gtk.Button()
            button.connect("clicked", self.on_call_service, command)

            label = Gtk.Label(label)
            label.modify_font(Pango.FontDescription('Sans 20'))
            label.set_padding(30, 30)
            button.add(label)
            label.show()

            self.box.pack_start(button, True, True, 0)

    def on_call_service(self, widget, service_name):
        rospy.loginfo('Calling {}'.format(service_name))
        self.service(service_name, '')

def main():
    rospy.init_node('skydio_gui', anonymous=False)

    # Allow the node to be killed with ^C
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    win = SkydioServiceWindow()
    win.connect("destroy", Gtk.main_quit)
    win.show_all()
    Gtk.main()


if __name__ == '__main__':
    main()
