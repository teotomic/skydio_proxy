"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class VectorXf(object):
    __slots__ = ["rows", "data"]

    def __init__(self,
                 rows=0,
                 data=None,
                 _skip_initialize=False):
        """ If _skip_initialize is True, all other constructor arguments are ignored """
        if _skip_initialize: return
        self.rows = rows
        self.data = [] if data is None else data

    @staticmethod
    def _skytype_meta():
        return dict(
            type='struct',
            package='eigen_lcm',
            name='VectorXf',
        )

    @classmethod
    def _default(cls):
        return cls()

    def __repr__(self):
        return 'lcmtypes.eigen_lcm.VectorXf({})'.format(
            ', '.join('{}={}'.format(name, repr(getattr(self, name))) for name in self.__slots__))

    def encode(self):
        buf = BytesIO()
        buf.write(VectorXf._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.rows))
        buf.write(struct.pack('>%df' % self.rows, *self.data[:self.rows]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != VectorXf._get_packed_fingerprint():
            raise ValueError("Decode error")
        return VectorXf._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = VectorXf(_skip_initialize=True)
        self.rows = struct.unpack(">i", buf.read(4))[0]
        self.data = struct.unpack('>%df' % self.rows, buf.read(self.rows * 4))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if VectorXf in parents: return 0
        tmphash = (0xd44af9becfc4d42a) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if VectorXf._packed_fingerprint is None:
            VectorXf._packed_fingerprint = struct.pack(">Q", VectorXf._get_hash_recursive([]))
        return VectorXf._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
