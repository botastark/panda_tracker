from __future__ import annotations

from backends.panda_udp import PandaUdpBackend


class MujocoUdpBackend(PandaUdpBackend):
    """
    The MuJoCo simulator intentionally mirrors the real Panda UDP interface.

    Therefore this backend is the same transport as PandaUdpBackend, but the
    destination is normally localhost.
    """
