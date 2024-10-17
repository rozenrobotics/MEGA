class PDCompass:
    def __init__(
        self,
        k_p: float | int = 1,
        k_d: float | int = 1,
        setpoint: float | int = 0,
    ) -> None:
        self.k_p = k_p
        self.k_d = k_d
        self.setpoint: float | int = setpoint

        self.last_error: float = 0.0
        self.error = 0

    @property
    def U(self) -> float:
        return self._u

    def tick(self, new_value: float | int, bypass: int = 5) -> "PDCompass":
        error = self.__get_error(new_value, bypass)

        derivative = error - self.last_error

        P = self.k_p * error
        D = self.k_d * derivative

        self._u = P + D

        self.last_error = error

        return self

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.error_sum = 0.0

    def __get_error(self, new_value: float | int, bypass: int) -> int | float:
        self.error = 0
        if self.setpoint > new_value:
            self.error = self.setpoint - new_value
        else:
            self.error = 360 + self.setpoint - new_value

        if self.error > 180:
            self.error = self.error - 360 

        if abs(self.error) < bypass:
            return 0

        return self.error
