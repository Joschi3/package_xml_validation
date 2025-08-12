import logging
import sys

logging.basicConfig(
    level=logging.DEBUG,  # show all levels
    format="%(message)s",
    stream=sys.stdout,  # important for CI
    force=True,  # override any previous logging config
)
