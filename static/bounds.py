from numpy import array


class Bounds:
    YELLOW_GATES = (
        array([0, 0, 0]),
        array([100, 100, 100]),
    )
    SPONGE = (
        array([125, 50, 0]),
        array([179, 255, 160]),
    )
    LITTER = (
        array([0, 66, 0]),
        array([20, 255, 255]),
    )
    # LITTER = (
    #     array([0, 47, 30]),
    #     array([25, 255, 255]),
    # )
    FINISH = (
        array([150, 0, 0]),
        array([180, 255, 200]),
    )
