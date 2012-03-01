from . import gray, np
from bootstrapping_olympics.utils import assert_allclose


def test_gray():
    for n in [4, 5, 6, 7, 8]:
        code = gray(n)
        check_gray_code(code)


def check_gray_code(code):
    for i in range(len(code) - 1):
        w1 = code[i]
        w2 = code[i + 1]
        dist = np.sum(np.abs(w1 - w2))
        assert_allclose(dist, 1)
