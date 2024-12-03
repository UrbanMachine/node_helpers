class DuplicateTransformError(Exception):
    """Raised when a transform is created that already exists"""


class MultipleParentsError(Exception):
    """Raised when a child TF is configured to have multiple parents"""
