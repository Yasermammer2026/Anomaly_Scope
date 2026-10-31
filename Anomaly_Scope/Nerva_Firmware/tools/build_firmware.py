"""Build the exported SDK in an isolated temporary copy. Never accesses a board."""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PROJECT = Path('Projects/STM32U585AI-STWIN.box/Applications/DATALOG2/STM32CubeIDE')


def locate_headless(explicit):
    if explicit or os.environ.get('STM32CUBEIDE_HEADLESS'):
        p = Path(explicit or os.environ['STM32CUBEIDE_HEADLESS']).resolve(strict=True)
        if not p.is_file():
            raise RuntimeError('Headless build launcher must be a file')
        return p
    for name in ('headless-build.bat', 'headless-build.sh'):
        found = shutil.which(name)
        if found:
            return Path(found).resolve()
    matches = sorted(Path('C:/ST').glob('STM32CubeIDE*/STM32CubeIDE/headless-build.bat'))
    if matches:
        return matches[-1].resolve()
    raise RuntimeError('Supply --headless-build or STM32CUBEIDE_HEADLESS')


def audit_dependencies(release, sdk, gcc):
    sdk = sdk.resolve()
    tools = gcc.parent.parent.resolve()
    found, external, missing = set(), set(), set()
    depfiles = list(release.rglob('*.d'))
    if not depfiles:
        raise RuntimeError('No fresh compiler dependency files')
    for dep in depfiles:
        body = dep.read_text(errors='replace').replace('\\\n', ' ').partition(':')[2]
        for token in re.findall(r'(?:\\.|[^\s])+', body):
            if token.endswith(':'):
                continue
            token = re.sub(r'\\([ :#$\\])', r'\1', token)
            p = Path(token)
            p = (p if p.is_absolute() else release / p).resolve()
            if not p.is_file():
                missing.add(str(p))
            elif p.is_relative_to(sdk):
                found.add(p.relative_to(sdk).as_posix())
            elif not p.is_relative_to(tools):
                external.add(str(p))
    if missing or external:
        raise RuntimeError('Dependency isolation failed: ' + json.dumps(
            dict(missing=sorted(missing)[:10], outside_sdk_or_compiler=sorted(external)[:10])))
    return dict(dependency_files=len(depfiles), sdk_inputs=sorted(found),
                missing_dependencies=0, original_project_dependencies=0,
                external_dependencies='Only installed compiler standard headers/libraries')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless-build')
    args = p.parse_args()
    headless = locate_headless(args.headless_build)
    gcc_name = 'arm-none-eabi-gcc.exe' if os.name == 'nt' else 'arm-none-eabi-gcc'
    compilers = sorted(headless.parent.rglob(gcc_name))
    if not compilers:
        raise RuntimeError('Install the STM32CubeIDE bundled GNU compiler')
    gcc = compilers[-1]
    version = subprocess.check_output([str(gcc), '--version'], text=True).splitlines()[0]
    verification = ROOT / 'verification'
    verification.mkdir(exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix='nerva-build-')).resolve()
    try:
        sdk = work / 'SDK'
        print('Staging a clean, independent copy of the exported SDK...', flush=True)
        shutil.copytree(ROOT / 'SDK', sdk)
        project = sdk / PROJECT
        workspace = work / 'workspace'
        log = work / 'build.log'
        command = [str(headless), '-no-indexer', '-data', str(workspace),
                   '-import', str(project), '-cleanBuild', 'DATALOG2-STWIN.box/Release']
        print('Building Release; no flashing or debugger operation is performed.', flush=True)
        with log.open('w', encoding='utf-8') as stream:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=1800)
        text = log.read_text(errors='replace')
        diagnostics = [v for v in text.splitlines() if re.search(r'\b(?:error|warning):', v)]
        release = project / 'Release'
        elf = release / 'DATALOG2-STWIN.box.elf'
        binary = release / 'DATALOG2-STWIN.box.bin'
        if (result.returncode or not elf.is_file() or not binary.is_file()
                or not re.search(r'Build Finished\. 0 errors,', text)
                or any(re.search(r'\berror:', v) for v in diagnostics)):
            (verification / 'build-failure.txt').write_text(text, encoding='utf-8')
            raise RuntimeError('Build failed; see verification/build-failure.txt')
        dependency = audit_dependencies(release, sdk, gcc)
        artifacts = ROOT / 'firmware'
        artifacts.mkdir(exist_ok=True)
        hashes = {}
        for source in (elf, binary, release / 'DATALOG2-STWIN.box.map'):
            if source.is_file():
                target = artifacts / source.name
                shutil.copy2(source, target)
                hashes[target.name] = hashlib.sha256(target.read_bytes()).hexdigest()
        size_name = 'arm-none-eabi-size.exe' if os.name == 'nt' else 'arm-none-eabi-size'
        size = subprocess.check_output([str(gcc.with_name(size_name)), str(elf)], text=True)
        size = size.replace(str(elf), 'DATALOG2-STWIN.box.elf')
        (verification / 'size.txt').write_text(size, encoding='utf-8')
        report = dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(), success=True,
                      configuration='Release', compiler=version,
                      warnings=sum(bool(re.search(r'\bwarning:', v)) for v in diagnostics),
                      errors=0, isolated_relocated_build=True,
                      board_access=False, original_firmware_modified=False,
                      original_verified_image_reproduced_exactly=False,
                      reason='Private credentials deliberately removed; rebuild paths also change ELF debug information.',
                      hardware_validation='Not performed for the credential-free rebuilt image',
                      artifacts_SHA256=hashes, dependency_audit=dependency)
        (verification / 'build-verification.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'dependency_audit'}, indent=2), flush=True)
    finally:
        # Only remove the absolute temporary directory created by this invocation.
        parent = Path(tempfile.gettempdir()).resolve()
        if work.parent != parent or not work.name.startswith('nerva-build-'):
            raise RuntimeError('Refusing cleanup outside the verified temporary build directory')
        shutil.rmtree(work)


if __name__ == '__main__':
    main()
