"""HAL package. Contract lives in hal/hal.py (import as `from hal.hal import ...` or `from hal import ...`)."""
from .hal import HAL_CONTRACT_VERSION, Hal, HalFault, ImuSample, EncoderSample  # noqa: F401

__all__ = ["HAL_CONTRACT_VERSION", "HalFault", "ImuSample", "EncoderSample", "Hal"]
