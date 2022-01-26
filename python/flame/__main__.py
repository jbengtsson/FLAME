from __future__ import print_function
from . import _pyapi_version, _capi_version, __version__

fmt = """This is FLAME

  FLAME version:      {fv}
  Python API version: {version}
  C++    API version: {cversion}
"""
print(fmt.format(fv=__version__, version=_pyapi_version, cversion=_capi_version))
