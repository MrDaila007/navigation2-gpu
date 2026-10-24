#!/usr/bin/env python3
"""Validate the TASK-005 planning baseline; not R2 runtime acceptance."""
from pathlib import Path
import hashlib
import re
import subprocess

root = Path(__file__).resolve().parents[2]
paths = [root / 'specs/MANIFEST.md', *sorted(Path(__file__).parent.glob('*.md'))]
links = 0
for path in paths:
    content = path.read_text()
    assert content.endswith('\n'), path
    assert all(line.rstrip() == line for line in content.splitlines()), path
    assert len(re.findall(r'^```', content, re.M)) % 2 == 0, path
    for target in re.findall(r'\]\(([^)]+)\)', content):
        if '://' not in target and not target.startswith('#'):
            assert (path.parent / target.split('#')[0]).exists(), (path, target)
            links += 1
    for block in re.findall(r'```bash\n(.*?)```', content, re.S):
        subprocess.run(['bash', '-n'], input=block, text=True, check=True)
plan = (Path(__file__).parent / 'writing-plans-plan.md').read_text()
for number in range(1, 11):
    assert f'| R2-{number:02d} ' in plan
for number in range(7):
    assert f'## P{number} ' in plan
manifest = root / 'tasks/TASK-002/systematic-debugger-artifacts/source-hashes.txt'
for line in manifest.read_text().splitlines():
    digest, name = line.split()
    assert hashlib.sha256((root / 'nav2_mppi_controller' / name).read_bytes()).hexdigest() == digest, name
anchors = {
    'src/optimizer.cpp': ['Optimizer::initialize', 'Optimizer::shutdown', 'Optimizer::reset',
                          'Optimizer::optimize', 'validator_loader_', 'generateNoisedTrajectories'],
    'src/critic_manager.cpp': ['CriticManager::loadCritics', 'CriticManager::evalTrajectoriesScores'],
    'src/critics/cost_critic_cuda.cu': ['CostCriticCuda::DeviceState', 'grow_n', 'fail_after_d2h'],
    'test/optimizer_unit_tests.cpp': ['OptimizerTester', 'MotionModelLoaderOutlives'],
    'test/critic_manager_test.cpp': ['DummyCritic', 'CriticManagerWrapper'],
}
for name, symbols in anchors.items():
    source = (root / 'nav2_mppi_controller' / name).read_text()
    for symbol in symbols:
        assert symbol in source, (name, symbol)
subprocess.run(['git', 'diff', '--check'], cwd=root, check=True)
subprocess.run(['git', 'diff', '--exit-code', 'b23f498a', '--', 'nav2_mppi_controller'],
               cwd=root, check=True)
print(f'PASS: {len(paths)} Markdown files, {links} local links, shell syntax, '
      '10 requirements, 7 checkpoints, 8 source hashes, 15 source anchors; no production diff')
