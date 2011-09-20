from bootstrapping_olympics.interfaces import BootSpec

def test_equality():
    s1 = BootSpec((180,), [(-1, 1), (-1, 1)])
    s2 = BootSpec((180,), [(-1, 1), (-1, 1)])
    assert s1 == s2, "%s != %s" % (s1, s2)
