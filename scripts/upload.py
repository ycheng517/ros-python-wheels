import fire
from ros_python_wheels.upload_wheels import upload_wheels_to_cloudsmith


if __name__ == "__main__":
    fire.Fire(upload_wheels_to_cloudsmith)
