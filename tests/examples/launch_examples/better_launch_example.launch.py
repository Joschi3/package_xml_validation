import better_launch as bl


def generate_launch_description():
    # bl.node("ignored_pkg", "ignored_exec")
    bl.node("bl_pkg_a", "talker")
    bl.include("bl_pkg_b", "child.launch.py")
