"""Build the simulator's test programs (fw_tests/t_*.c) with the GCC that ships with CCS.

Usage: python tools/build_fw.py [--ccs C:\\ti\\ccsv7]
Outputs fw_tests/build/<name>.elf.  Returns non-zero if any build fails.
"""
import argparse
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, "..", "fw_tests")


def find_ccs(hint=None):
    cands = [hint] if hint else []
    cands += [os.environ.get("CCS_ROOT", "")]
    for base in (r"C:\ti", "/opt/ti", os.path.expanduser("~/ti")):
        cands += sorted(glob.glob(os.path.join(base, "ccs*", "ccs")), reverse=True)
        cands += sorted(glob.glob(os.path.join(base, "ccsv*")), reverse=True)
    for c in cands:
        if c and glob.glob(os.path.join(c, "tools", "compiler", "gcc-arm-none-eabi*", "bin")):
            return c
    return None


def toolchain(ccs):
    gcc_dir = sorted(glob.glob(os.path.join(ccs, "tools", "compiler", "gcc-arm-none-eabi*", "bin")))[-1]
    exe = ".exe" if os.name == "nt" else ""
    return os.path.join(gcc_dir, "arm-none-eabi-gcc" + exe)


def build(ccs, names=None, quiet=False):
    gcc = toolchain(ccs)
    inc = os.path.join(ccs, "ccs_base", "arm", "include")
    src = os.path.join(ccs, "ccs_base", "arm", "src")
    out_dir = os.path.join(FW, "build")
    os.makedirs(out_dir, exist_ok=True)
    sources = sorted(glob.glob(os.path.join(FW, "t_*.c")))
    if names:
        sources = [s for s in sources if os.path.splitext(os.path.basename(s))[0] in names]
    ok = True
    for c in sources:
        name = os.path.splitext(os.path.basename(c))[0]
        elf = os.path.join(out_dir, name + ".elf")
        deps = [c, os.path.join(FW, "rslk_hw.h")]
        if os.path.exists(elf) and all(os.path.getmtime(elf) >= os.path.getmtime(d) for d in deps):
            continue
        cmd = [gcc, "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=hard", "-mfpu=fpv4-sp-d16",
               "-O1", "-g", "-D__MSP432P401R__", "-Dgcc", f"-I{inc}", f"-I{os.path.join(inc, 'CMSIS')}",
               "-ffunction-sections", "-fdata-sections", "-Wall", "-Wno-unused-function",
               c, os.path.join(src, "startup_msp432p401r_gcc.c"), os.path.join(src, "system_msp432p401r.c"),
               "-T", os.path.join(inc, "msp432p401r.lds"), "--specs=nosys.specs", "-Wl,--gc-sections",
               "-o", elf]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            ok = False
            print(f"FAILED {name}:\n{r.stderr}", file=sys.stderr)
        elif not quiet:
            print(f"built {os.path.relpath(elf)}")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ccs", help="CCS install folder (default: search C:\\ti)")
    ap.add_argument("names", nargs="*")
    a = ap.parse_args()
    ccs = find_ccs(a.ccs)
    if not ccs:
        sys.exit("Code Composer Studio with the GCC ARM compiler was not found (use --ccs)")
    sys.exit(0 if build(ccs, a.names) else 1)


if __name__ == "__main__":
    main()
