import logging
import os
import sys


class ColoredFormatter(logging.Formatter):
    """
    A formatter that adds color to log levels.
    """

    # ANSI escape sequences for colors
    RESET = "\033[0m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"

    # You can map each log level to a specific color
    LOG_COLORS = {
        logging.DEBUG: GREEN,
        logging.INFO: BLUE,
        logging.WARNING: YELLOW,
        logging.ERROR: RED,
        logging.CRITICAL: RED,
    }

    def __init__(self, *args, use_color: bool = True, **kwargs):
        """Initialize the formatter with optional ANSI coloring.

        Args:
            *args: Positional args forwarded to logging.Formatter.
            use_color: Whether to wrap formatted output in ANSI color codes.
            **kwargs: Keyword args forwarded to logging.Formatter.

        Returns:
            None.

        """
        super().__init__(*args, **kwargs)
        self.use_color = use_color

    def format(self, record):
        """Format a log record and optionally colorize the output.

        Args:
            record: Log record to format.

        Returns:
            The formatted log string (possibly colorized).

        """
        formatted = super().format(record)
        indent = "\t" if record.levelno == logging.ERROR else ""
        if not self.use_color:
            return f"{indent}{formatted}"
        log_color = self.LOG_COLORS.get(record.levelno, self.RESET)
        return f"{indent}{log_color}{formatted}{self.RESET}"


def _resolve_level(level: str | int) -> int:
    """Resolve a level specifier to a logging level integer.

    Args:
        level: Either a level name ("normal", "verbose", "INFO") or a level int.

    Returns:
        The corresponding logging level integer.

    """
    if isinstance(level, int):
        return level
    if level == "verbose":
        return logging.DEBUG
    if level == "normal":
        return logging.INFO
    return logging._nameToLevel.get(level.upper(), logging.INFO)


def get_logger(name: str = __name__, level: str | int = "normal") -> logging.Logger:
    """Create or retrieve a configured logger with a stream handler.

    Args:
        name: Logger name.
        level: "normal"/"verbose", a logging level name, or a level int.

    Returns:
        A configured logger instance.

    """
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    ch = logging.StreamHandler(sys.stdout)  # stdout is better for CI
    ch.flush = sys.stdout.flush  # force flush after each log

    ch.setLevel(_resolve_level(level))

    is_tty = sys.stdout.isatty()
    use_color = is_tty and not os.getenv("NO_COLOR")
    formatter = ColoredFormatter("%(message)s", use_color=use_color)
    ch.setFormatter(formatter)

    if not any(
        isinstance(handler, logging.StreamHandler) for handler in logger.handlers
    ):
        logger.addHandler(ch)

    logger.propagate = False  # don't let root logger duplicate lines
    return logger


# Example Usage
# if __name__ == "__main__":
#     # Example usage:
#     # For "normal" logs (INFO and above)
#     logger_normal = get_logger(level="normal")
#     logger_normal.debug("Debug (won't show at 'normal' level)")
#     logger_normal.info("Info message")
#     logger_normal.warning("Warning message")
#     logger_normal.error("Error message")

#     print("\n--- Now with verbose logs ---\n")

#     # For "verbose" logs (DEBUG and above)
#     logger_verbose = get_logger(level="verbose")
#     logger_verbose.debug("Debug (visible at 'verbose' level)")
#     logger_verbose.info("Info message")
#     logger_verbose.warning("Warning message")
#     logger_verbose.error("Error message")
