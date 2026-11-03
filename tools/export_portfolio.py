#!/usr/bin/env python3
"""Export source files without changing nested Git repositories."""
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {
    '.git', '.agents', '.codex', '.aws', '.vscode', '.idea',
    '__pycache__', '.pytest_cache', '.venv', 'venv',
}


def main():
    output = ROOT / 'dist' / 'turtlebot3-portfolio.zip'
    output.parent.mkdir(exist_ok=True)
    count = 0
    with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
        for base in ('src', 'docs', 'tools'):
            for path in sorted((ROOT / base).rglob('*')):
                rel = path.relative_to(ROOT)
                if any(part in EXCLUDED or part.endswith('.egg-info') for part in rel.parts):
                    continue
                if path.is_symlink():
                    raise RuntimeError(f'Review symlink before export: {rel}')
                if not path.is_file():
                    continue
                if (path.name.startswith('.env') or path.name.startswith('~$')
                        or path.suffix in {'.pyc', '.pyo', '.pem', '.key', '.db3', '.mcap'}):
                    continue
                archive.write(path, rel)
                count += 1
        for name in ('README.md', '.gitignore'):
            archive.write(ROOT / name, name)
            count += 1
    print(f'{output}: {count} files, {output.stat().st_size / 1024**2:.1f} MiB')


if __name__ == '__main__':
    main()
