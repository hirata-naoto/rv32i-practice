# rv32i-practice

合成可能な読みやすさ重視の RV32I コアを `rtl/rv32i_core.sv` に追加しています。

## `rv32i_core` の概要

- 32bit RV32I の整数命令を対象にしたシンプルなマルチサイクル実装です
- `FETCH -> DECODE -> EXECUTE -> MEMORY -> WRITEBACK` の順に進むため、速度より見通しを優先しています
- 命令メモリとデータメモリを分けたシンプルなインターフェースを採用しています
- 不正命令、未サポートの `SYSTEM` 命令、アラインメント違反を検出すると `trap` を立てて停止します

## インターフェース

- `imem_addr` / `imem_rdata`
  - 命令フェッチ用の単純な読み出しポートです
  - `imem_addr` に対する命令が同サイクルに `imem_rdata` で得られる前提です
- `dmem_valid` / `dmem_addr` / `dmem_wdata` / `dmem_wstrb` / `dmem_rdata`
  - データメモリアクセス用のポートです
  - 読み出し時は `dmem_wstrb == 4'b0000`、書き込み時はバイトイネーブルを `dmem_wstrb` に出力します
  - データメモリも待ちなしで同サイクルに応答する前提です
- `trap`
  - コアが停止状態に入ると High のまま維持されます

## 補足

- レジスタ `x0` は常に 0 に固定されています
- `FENCE` / `FENCE.I` はメモリバリア動作を持たない NOP 相当として扱います
- `ECALL` / `EBREAK` などの `SYSTEM` 命令は `trap` になります