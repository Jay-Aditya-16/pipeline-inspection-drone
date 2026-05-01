"""
safety.py
Rule-based safety classification for pipeline inspection system.

Alert levels:
    SAFE    -- all gas concentrations below 50% of LEL
    WARNING -- any gas concentration between 50% and 100% of LEL
    DANGER  -- any gas concentration at or above LEL, OR inactive worker detected

Lower Explosive Limits (LEL) for reference:
    LPG/Propane : ~2100 ppm (0.21% v/v)
    Methane     : ~5000 ppm (0.50% v/v)
    Hydrogen    : ~4000 ppm (0.40% v/v)
    CO          :  ~200 ppm IDLH (not flammable at typical levels but toxic)
    Ammonia     : ~150,000 ppm LEL (but IDLH = 300 ppm)
    Alcohol     : ~3300 ppm
"""


class SafetyMonitor:
    """
    Classifies environmental safety status from sensor readings.

    Args:
        thresholds: dict mapping gas key -> danger threshold (ppm)
                    Defaults to 50% of LEL for explosive gases,
                    IDLH for toxic gases.
    """

    def __init__(self, thresholds: dict):
        self.thresholds = thresholds

    def classify(self, sensor_data: dict, worker_alert: bool = False) -> str:
        """
        Returns 'SAFE', 'WARNING', or 'DANGER'.

        ALERT condition (DANGER):
            posture == prone  AND  C_gas > tau  AND  delta_p < epsilon
        Combined with threshold-based alert for gas readings.
        """
        if worker_alert:
            return "DANGER"

        danger  = False
        warning = False

        for gas, threshold in self.thresholds.items():
            val = sensor_data.get(gas, 0)
            if val >= threshold:
                danger = True
                break
            elif val >= threshold * 0.5:
                warning = True

        if danger:
            return "DANGER"
        if warning:
            return "WARNING"
        return "SAFE"

    def above_threshold(self, sensor_data: dict) -> list[str]:
        """Returns list of gas keys currently above their danger threshold."""
        return [gas for gas, thr in self.thresholds.items()
                if sensor_data.get(gas, 0) >= thr]
