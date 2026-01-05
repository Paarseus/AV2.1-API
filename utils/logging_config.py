"""
Centralized Logging Configuration for UTM Navigator

Usage:
    from utils.logging_config import setup_logging, get_logger

    # At application startup
    setup_logging(level='INFO')  # or 'DEBUG' for development

    # In any module
    logger = get_logger(__name__)
    logger.info("Message here")
"""

import logging
import sys
from typing import Optional


# Default format for log messages
DEFAULT_FORMAT = "[%(asctime)s] %(levelname)-8s %(name)-20s: %(message)s"
DEFAULT_DATE_FORMAT = "%H:%M:%S"

# Module-specific log levels (can be adjusted)
MODULE_LEVELS = {
    "sensors": logging.INFO,
    "perception": logging.INFO,
    "planning": logging.INFO,
    "control": logging.INFO,
    "actuators": logging.WARNING,  # Less verbose for actuators
}


def setup_logging(
    level: str = "INFO",
    log_file: Optional[str] = None,
    format_string: str = DEFAULT_FORMAT,
    date_format: str = DEFAULT_DATE_FORMAT
) -> None:
    """
    Configure logging for the entire application.

    Args:
        level: Root log level ('DEBUG', 'INFO', 'WARNING', 'ERROR')
        log_file: Optional path to log file. If None, logs to stderr only.
        format_string: Log message format
        date_format: Timestamp format
    """
    # Convert string level to logging constant
    numeric_level = getattr(logging, level.upper(), logging.INFO)

    # Create formatter
    formatter = logging.Formatter(format_string, datefmt=date_format)

    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(numeric_level)

    # Remove existing handlers
    root_logger.handlers.clear()

    # Console handler (stderr)
    console_handler = logging.StreamHandler(sys.stderr)
    console_handler.setLevel(numeric_level)
    console_handler.setFormatter(formatter)
    root_logger.addHandler(console_handler)

    # File handler (optional)
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(numeric_level)
        file_handler.setFormatter(formatter)
        root_logger.addHandler(file_handler)

    # Apply module-specific levels
    for module, mod_level in MODULE_LEVELS.items():
        logging.getLogger(module).setLevel(max(mod_level, numeric_level))


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger for the specified module.

    Args:
        name: Module name (typically __name__)

    Returns:
        Configured logger instance
    """
    return logging.getLogger(name)


# Convenience function for quick setup
def setup_debug():
    """Quick setup for debug mode with verbose output."""
    setup_logging(level="DEBUG")


def setup_production():
    """Quick setup for production with minimal output."""
    setup_logging(level="WARNING")
