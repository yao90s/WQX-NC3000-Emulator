# Builds the emulator with the portable mingw-w64 toolchain in ../toolchain.
#
# 默认（x64）：toolchain/mingw64 + SDL2 2.32.10 → nc3000.exe
# -XP（32 位 WinXP）：toolchain/mingw32(i686/msvcrt) + SDL2 2.0.22 → nc3000_xp.exe
#   XP 版必须满足：32 位、msvcrt 运行时（不能用 api-ms-win-crt-*）、
#   SDL2 只能用 2.0.22 或更早（2.24+ 放弃 XP）、_WIN32_WINNT=0x0501。
param(
    [string]$Target = '',
    [switch]$Debug,
    [switch]$Headless,
    [switch]$XP,
    [string[]]$Extra
)
$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$tc   = Join-Path (Split-Path -Parent $root) 'toolchain'
$spce = Join-Path (Split-Path -Parent $root) 'spce061a'

if ($XP) {
    $gxx = Join-Path $tc 'mingw32\bin\g++.exe'
    $gcc = Join-Path $tc 'mingw32\bin\gcc.exe'
    $sdl = Join-Path $tc 'sdl2_xp\i686-w64-mingw32'
    $objDir = Join-Path $root 'obj_xp32'
    # 目标系统 = Windows XP（5.1），别用 XP 之后才有的 API
    $osFlags = @('-D_WIN32_WINNT=0x0501', '-DWINVER=0x0501', '-DNTDDI_VERSION=0x05010000')
    $defaultTarget = if ($Headless) { 'nc3000_xp_headless.exe' } else { 'nc3000_xp.exe' }
} else {
    $gxx = Join-Path $tc 'mingw64\mingw64\bin\g++.exe'
    $gcc = Join-Path $tc 'mingw64\mingw64\bin\gcc.exe'
    $sdl = Join-Path $tc 'sdl2\SDL2-2.32.10\x86_64-w64-mingw32'
    $objDir = Join-Path $root 'obj'
    $osFlags = @()
    $defaultTarget = if ($Headless) { 'nc3000_headless.exe' } else { 'nc3000.exe' }
}
if ($Target -eq '') { $Target = $defaultTarget }

if (!(Test-Path $gxx)) { throw "compiler not found: $gxx" }
if (!(Test-Path (Join-Path $sdl 'include\SDL2\SDL.h'))) { throw "SDL2 not found in $sdl" }

$coreSources = @(
    'nc2000.cpp',
    'misc/disassembler.cpp', 'misc/disassembler_new.cpp', 'misc/bin_dec.cpp',
    'misc/udp_server.cpp',
    'misc/irda_link.cpp',
    'cpu_loop.cpp', 'cpu_loop_new.cpp', 'cpu.cpp', 'comm.cpp',
    'mem.cpp', 'io.cpp', 'io_new.cpp', 'rom.cpp', 'nor.cpp', 'nand.cpp', 'ram.cpp',
    'dsp/dsp.cpp', 'sound.cpp', 'cmd.cpp', 'console.cpp',
    'ansi/w65c02cpu.cpp', 'ansi/w65c02op.cpp',
    'NekoDriverIO.cpp', 'key_new.cpp', 'key.cpp',
    'compare/c6502.cpp', 'compare/pc1000bus.cpp',
    'settings.cpp',
    'spce061_bridge.cpp',
    'iv_uart.cpp'
)

# the SPCE061A emulator is plain C99 and is compiled separately with gcc
$cSources = @(
    (Join-Path $spce 'emu\unsp.c'),
    (Join-Path $spce 'emu\unsp_jit_x64.c'),
    (Join-Path $spce 'emu\spce061a.c'),
    (Join-Path $spce 'emu\firmware_061.c'),
    (Join-Path $spce 'host\firmware_061_nand.c'),
    (Join-Path $spce 'host\nc3000_dsp.c')
)
$cFlags = @('-std=c99', '-O2', '-w',
            "-I$(Join-Path $spce 'emu')", "-I$(Join-Path $spce 'host')") + $osFlags

New-Item -ItemType Directory -Force -Path $objDir | Out-Null
$cObjects = @()
foreach ($c in $cSources) {
    if (!(Test-Path $c)) { throw "missing source: $c" }
    $o = Join-Path $objDir ((Split-Path -Leaf $c) + '.o')
    $cObjects += $o
    if ((Test-Path $o) -and ((Get-Item $o).LastWriteTime -gt (Get-Item $c).LastWriteTime)) { continue }
    & $gcc @cFlags -c $c -o $o
    if ($LASTEXITCODE -ne 0) { throw "failed to compile $c" }
}

if ($Headless) {
    $sources = @('headless/main_headless.cpp') + $coreSources
    $guiFlags = @()
} else {
    $sources = @('main.cpp', 'display.cpp',
                 'lcdstripe/json.cpp', 'lcdstripe/lcdpainter.cpp') + $coreSources
    # -mwindows：GUI 版编成 Windows 窗口子系统，双击时不再多出一个控制台窗口
    # （那个控制台会抢焦点，按键全被它吃掉）。要日志就加 --console 参数。
    $guiFlags = @('-mwindows')
}

$opt = if ($Debug) { '-O0', '-g3' } else { '-O2', '-g3' }
$cxxArgs = @(
    '-std=c++11', '-DHANDYPSP', '-pthread',
    '-Wno-deprecated-declarations', '-Wno-write-strings',
    '-Wno-narrowing', '-Wno-unused-variable', '-Wno-unused-but-set-variable'
    "-I$root", "-I$sdl/include", "-I$sdl/include/SDL2",
    "-I$(Join-Path $spce 'emu')", "-I$(Join-Path $spce 'host')"
) + $osFlags + $opt + @(
    # 静态链上 mingw 运行时（libgcc / libstdc++ / winpthread）：
    # 否则 exe 会从 libwinpthread-1.dll 导入 clock_gettime64，用户 PATH 上只要有一份
    # 更旧的同名 DLL 就会报"无法定位程序输入点 clock_gettime64"。
    '-static', '-static-libgcc', '-static-libstdc++'
) + $guiFlags + $Extra + $sources + $cObjects + @(
    '-o', (Join-Path $root $Target),
    "-L$sdl/lib", '-lmingw32', '-lSDL2main', '-lSDL2',
    # 上面 -static 会让 ld 选静态 libSDL2.a，于是要补 SDL2 依赖的系统库；
    # 好处是出来的 exe 连 SDL2.dll 都不需要，单文件就能跑。
    '-luser32', '-lgdi32', '-lwinmm', '-limm32', '-lole32', '-loleaut32',
    '-luuid', '-lversion', '-ladvapi32', '-lsetupapi', '-lshell32', '-lhid',
    # 红外主机桥（misc/irda_link.cpp）用 winsock2
    '-lws2_32'
)

Push-Location $root
try {
    & $gxx @cxxArgs
    if ($LASTEXITCODE -ne 0) { throw "build failed (exit $LASTEXITCODE)" }
    # 静态链上 SDL2 之后不需要 SDL2.dll；XP 版就不要再放一个 32 位 DLL 进来。
    if (!$XP) {
        Copy-Item -LiteralPath (Join-Path $sdl 'bin\SDL2.dll') -Destination $root -Force
    }
    Write-Host "built $Target"
} finally {
    Pop-Location
}
