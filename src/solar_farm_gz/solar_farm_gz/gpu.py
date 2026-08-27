"""NVIDIA PRIME render-offload detection.

Laptops with switchable graphics run the desktop on the integrated GPU and
leave the discrete card idle until a process explicitly asks for it. Gazebo
doesn't ask, so on a machine like an MSI GS66 or an HP OMEN it will happily
render a 1000-module world on the Intel UHD graphics while an RTX card sits
at 15 MB of memory used. Nothing warns you: it's just slower, and the only
visible symptom is `libEGL warning: failed to create dri2 screen` buried in
the log.

These variables are the documented activation path. They're only set when
an NVIDIA GPU is actually present, because forcing
__GLX_VENDOR_LIBRARY_NAME=nvidia on a machine without the driver breaks GLX
outright instead of falling back gracefully.
"""

import glob
import os

_EGL_NVIDIA_JSON = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"


def nvidia_present():
    """True if an NVIDIA driver is loaded and its EGL vendor file exists.

    Checks /proc instead of invoking nvidia-smi: this runs during launch
    description, and a subprocess there costs startup latency on every
    launch for a question a file check already answers.
    """
    if not os.path.exists(_EGL_NVIDIA_JSON):
        return False
    return bool(glob.glob("/proc/driver/nvidia/gpus/*"))


def offload_env():
    """Environment overrides that move rendering to the discrete GPU.

    Returns an empty dict when there's no NVIDIA GPU, so callers can apply
    it unconditionally and stay portable.
    """
    if not nvidia_present():
        return {}
    return {
        "__NV_PRIME_RENDER_OFFLOAD": "1",
        "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
        "__EGL_VENDOR_LIBRARY_FILENAMES": _EGL_NVIDIA_JSON,
    }
