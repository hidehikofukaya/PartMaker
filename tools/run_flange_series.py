"""フランジ専用チャンクを、DELMIAの再起動を挟みながら連続実行する(2026-08-26、SS18)。

CATIAセッションは時間とともに重くなる(SS5.1: 15.6->27.5秒/部品、メモリ795MB->1.78GB)。
ユーザーがリモートのため、再起動もこちらで行う(明示的な依頼あり)。

各チャンクで:
  1. DELMIAを終了 -> 起動 -> COM接続可能になるまで待つ
  2. run_flange_chunk.py を**別プロセスで**実行(1チャンクの失敗が全体を巻き込まない)

使い方:
  python tools/run_flange_series.py <ルート名> <チャンク数> <1チャンクの部品数> <基準seed>
                                    [開始チャンク番号] [クォータJSON]
例:
  python tools/run_flange_series.py flange01 5 100 20260840
"""
from __future__ import annotations

import pathlib
import subprocess
import sys
import time

DELMIA_EXE = r"D:\Dassault Systemes\B32\win_b64\code\bin\Delmia.exe"
TOOLS = pathlib.Path(__file__).resolve().parent


def delmia_alive() -> bool:
    try:
        import win32com.client
        app = win32com.client.GetActiveObject("DELMIA.Application")
        app.Documents.Count
        return True
    except Exception:
        return False


def restart_delmia(timeout_s: float = 900.0) -> bool:
    """DELMIAを終了して起動し直し、COMが応答するまで待つ。"""
    subprocess.run(["taskkill", "/F", "/IM", "Delmia.exe"],
                   capture_output=True, check=False)
    # プロセスが消えるまで待つ(消えないうちに起動すると2重起動になる)
    for _ in range(60):
        out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq Delmia.exe"],
                             capture_output=True, text=True, check=False)
        if "Delmia.exe" not in out.stdout:
            break
        time.sleep(2)
    time.sleep(3)
    subprocess.Popen([DELMIA_EXE], creationflags=subprocess.DETACHED_PROCESS
                     if hasattr(subprocess, "DETACHED_PROCESS") else 0)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if delmia_alive():
            return True
        time.sleep(10)
    return False


def main() -> None:
    root_name = sys.argv[1]
    n_chunks = int(sys.argv[2])
    per_chunk = int(sys.argv[3])
    base_seed = int(sys.argv[4])
    start_chunk = int(sys.argv[5]) if len(sys.argv) > 5 else 1
    quota_json = sys.argv[6] if len(sys.argv) > 6 else None

    done = 0
    for chunk in range(start_chunk, start_chunk + n_chunks):
        print(f"\n{'=' * 60}\n=== chunk_{chunk:02d} / {n_chunks}  DELMIA再起動中 ===",
              flush=True)
        t0 = time.time()
        if not restart_delmia():
            print("*** DELMIAが起動しない。中断 ***", flush=True)
            break
        print(f"    起動完了 ({time.time() - t0:.0f}秒)", flush=True)
        seed = base_seed + chunk * 1000
        cmd = [sys.executable, str(TOOLS / "run_flange_chunk.py"),
               root_name, str(chunk), str(per_chunk), str(seed)]
        if quota_json:
            cmd.append(quota_json)
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        sys.stdout.write(proc.stdout)
        if proc.returncode != 0:
            sys.stdout.write(proc.stderr[-2000:])
            print(f"*** chunk_{chunk:02d} 失敗(rc={proc.returncode})。次へ進む ***", flush=True)
            continue
        done += per_chunk
        print(f"=== chunk_{chunk:02d} 完了 [累計 {done}部品] ===", flush=True)

    print(f"\nSERIES DONE: {done}部品 / {n_chunks}チャンク中", flush=True)


if __name__ == "__main__":
    main()
