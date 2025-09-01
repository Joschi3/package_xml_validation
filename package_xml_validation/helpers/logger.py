import logging
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

    def format(self, record):
        log_color = self.LOG_COLORS.get(record.levelno, self.RESET)
        indent = "\t" if record.levelno == logging.ERROR else ""
        record.msg = f"{indent}{log_color}{record.msg}{self.RESET}"
        return super().format(record)


def get_logger(name: str = __name__, level: str = "normal") -> logging.Logger:
    logger = logging.getLogger(f"{name}_{level}")
    logger.setLevel(logging.DEBUG)

    ch = logging.StreamHandler(sys.stdout)  # stdout is better for CI
    ch.flush = sys.stdout.flush  # force flush after each log

    if level == "verbose":
        ch.setLevel(logging.DEBUG)
    else:
        ch.setLevel(logging.INFO)

    formatter = ColoredFormatter("%(message)s")
    ch.setFormatter(formatter)

    if not logger.handlers:
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
