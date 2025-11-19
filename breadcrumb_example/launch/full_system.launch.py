import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    l.include_launch_py("breadcrumb_example", "_hardware.launch.py")
    l.include_launch_py("breadcrumb_example", "_control.launch.py")

    return l
