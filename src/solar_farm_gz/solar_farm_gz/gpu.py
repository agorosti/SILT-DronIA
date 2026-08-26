"""Detección de render-offload de NVIDIA PRIME.

Los portátiles con gráficos conmutables ejecutan el escritorio en la GPU
integrada y dejan la tarjeta discreta inactiva hasta que un proceso la pide
explícitamente. Gazebo no la pide, así que en una máquina como un MSI GS66 o
un HP OMEN renderizará sin problema un mundo de 1000 módulos en los
gráficos Intel UHD mientras una tarjeta RTX se queda en 15 MB de memoria
usada. Nada te avisa: simplemente va más lento, y el único síntoma visible
es `libEGL warning: failed to create dri2 screen` enterrado en el log.

Estas variables son la vía de activación documentada. Solo se fijan cuando
hay realmente una GPU NVIDIA presente, porque forzar
__GLX_VENDOR_LIBRARY_NAME=nvidia en una máquina sin el driver rompe GLX por
completo en lugar de hacer un fallback.
"""

import glob
import os

_EGL_NVIDIA_JSON = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"


def nvidia_present():
    """True si hay un driver de NVIDIA cargado y existe su fichero EGL vendor.

    Comprueba /proc en lugar de invocar nvidia-smi: esto se ejecuta durante
    la descripción del lanzamiento, y un subproceso ahí cuesta latencia de
    arranque en cada lanzamiento para una pregunta que una comprobación de
    fichero ya responde.
    """
    if not os.path.exists(_EGL_NVIDIA_JSON):
        return False
    return bool(glob.glob("/proc/driver/nvidia/gpus/*"))


def offload_env():
    """Overrides de entorno que mueven el renderizado a la GPU discreta.

    Devuelve un diccionario vacío cuando no hay GPU NVIDIA, para que quien
    llame pueda aplicarlo sin condiciones y siga siendo portable.
    """
    if not nvidia_present():
        return {}
    return {
        "__NV_PRIME_RENDER_OFFLOAD": "1",
        "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
        "__EGL_VENDOR_LIBRARY_FILENAMES": _EGL_NVIDIA_JSON,
    }
