class SecurityError(Exception):
    """Raised when a security violation occurs."""
    pass

class LandingError(Exception):
    """Raised when precision landing fails."""
    pass