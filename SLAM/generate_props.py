#!/usr/bin/env python3
"""Generate Open3DExample.props from CMake-built example vcxproj (one-time helper)."""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RGBD = ROOT / "build/examples/cpp/OnlineSLAMRGBD.vcxproj"
RS = ROOT / "build/examples/cpp/OnlineSLAMRealSense.vcxproj"


def extract(xml: str, cfg: str):
    pat = (
        r"ItemDefinitionGroup Condition=\"'\$\(Configuration\)\|\$\(Platform\)'=='"
        + cfg
        + r"\|x64'\">.*?<ClCompile>(.*?)</ClCompile>.*?<Link>(.*?)</Link>"
    )
    m = re.search(pat, xml, re.S)
    if not m:
        return None
    cl, link = m.group(1), m.group(2)
    deps = re.search(r"<AdditionalDependencies>(.*?)</AdditionalDependencies>", link, re.S)
    libdirs = re.search(
        r"<AdditionalLibraryDirectories>(.*?)</AdditionalLibraryDirectories>", link, re.S
    )
    opts = re.search(r"<AdditionalOptions>(.*?)</AdditionalOptions>", link, re.S)
    return {
        "cl": cl,
        "deps": deps.group(1) if deps else "",
        "libdirs": libdirs.group(1) if libdirs else "",
        "opts": opts.group(1) if opts else "",
    }


def fix_paths(s: str) -> str:
    s = s.replace("D:\\study\\Open3D\\", "$(Open3DRoot)\\")
    s = s.replace("D:/study/Open3D/", "$(Open3DRoot)/")
    s = s.replace("..\\..\\", "$(BuildRoot)\\")
    return s


def main():
    rgbd_xml = RGBD.read_text(encoding="utf-8")
    rs_xml = RS.read_text(encoding="utf-8")
    rs_release = extract(rs_xml, "Release")
    rs_debug = extract(rs_xml, "Debug")

    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<Project ToolsVersion="4.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">',
        "  <PropertyGroup Label=\"UserMacros\">",
        "    <Open3DRoot Condition=\"'$(Open3DRoot)'==''\">$(MSBuildThisFileDirectory)..</Open3DRoot>",
        "    <BuildRoot>$(Open3DRoot)\\build</BuildRoot>",
        "  </PropertyGroup>",
        "  <PropertyGroup>",
        "    <WindowsTargetPlatformVersion>10.0</WindowsTargetPlatformVersion>",
        "  </PropertyGroup>",
    ]

    for cfg in ["Debug", "Release"]:
        block = extract(rgbd_xml, cfg)
        if not block:
            raise SystemExit(f"Missing {cfg} block in {RGBD}")
        cl = fix_paths(block["cl"])
        deps = fix_paths(block["deps"])
        libdirs = fix_paths(block["libdirs"])
        opts = fix_paths(block["opts"])
        lines.extend(
            [
                f"  <ItemDefinitionGroup Condition=\"'$(Configuration)|$(Platform)'=='{cfg}|x64'\">",
                "    <ClCompile>",
            ]
        )
        for line in cl.strip().splitlines():
            lines.append("      " + line.strip())
        lines.extend(
            [
                "    </ClCompile>",
                "    <Link>",
                f"      <AdditionalDependencies>{deps}</AdditionalDependencies>",
                f"      <AdditionalLibraryDirectories>{libdirs}</AdditionalLibraryDirectories>",
                f"      <AdditionalOptions>{opts}</AdditionalOptions>",
                "      <SubSystem>Console</SubSystem>",
            ]
        )
        if cfg == "Debug":
            lines.append("      <GenerateDebugInformation>true</GenerateDebugInformation>")
        else:
            lines.append("      <GenerateDebugInformation>false</GenerateDebugInformation>")
        lines.extend(["    </Link>", "  </ItemDefinitionGroup>"])

    # RealSense extra include + libs (Release/Debug deltas from RealSense vcxproj).
    for cfg, rs_block in [("Release", rs_release), ("Debug", rs_debug)]:
        if not rs_block:
            continue
        rgbd_block = extract(rgbd_xml, cfg)
        extra_deps = set(rs_block["deps"].split(";")) - set(rgbd_block["deps"].split(";"))
        extra_deps.discard("")
        if extra_deps:
            lines.append(f"  <!-- RealSense extra libs for {cfg}: {', '.join(sorted(extra_deps))} -->")

    lines.append("</Project>")
    out = Path(__file__).resolve().parent / "Open3DExample.props"
    text = "\n".join(lines) + "\n"
    # SLAM local sources live under SLAM/cpp (not examples/cpp).
    text = text.replace(
        "$(Open3DRoot)\\examples\\cpp;",
        "$(MSBuildThisFileDirectory)cpp;",
    )
    text = text.replace(
        "$(Open3DRoot)/examples/cpp;",
        "$(MSBuildThisFileDirectory)cpp;",
    )
    out.write_text(text, encoding="utf-8")
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
