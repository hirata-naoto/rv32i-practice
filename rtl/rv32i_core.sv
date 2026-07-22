// RV32I プロセッサコア
// RISC-V 基本整数命令セット（32ビット）を実装した5ステージパイプラインレス（多サイクル）コアです。
// フェッチ・デコード・実行・メモリ・ライトバックの各ステートを順に遷移します。
module rv32i_core #(
    parameter logic [31:0] RESET_PC = 32'h0000_0000  // リセット時のプログラムカウンタ初期値
) (
    input  logic        clk,           // クロック（立ち上がりエッジで動作）
    input  logic        rst_n,         // 非同期リセット（アクティブLow）
    output logic [31:0] imem_addr,     // 命令メモリアドレス（現在のPC値）
    input  logic [31:0] imem_rdata,    // 命令メモリ読み出しデータ
    output logic        dmem_valid,    // データメモリアクセス有効フラグ
    output logic [31:0] dmem_addr,     // データメモリアドレス
    output logic [31:0] dmem_wdata,    // データメモリ書き込みデータ
    output logic [3:0]  dmem_wstrb,    // データメモリ書き込みストローブ（バイトイネーブル）
    input  logic [31:0] dmem_rdata,    // データメモリ読み出しデータ
    output logic        trap           // 例外・不正命令検出フラグ
);

    // -----------------------------------------------------------------------
    // RV32I オペコード定数（命令[6:0]）
    // -----------------------------------------------------------------------
    localparam logic [6:0] OPCODE_LUI    = 7'b0110111;  // Load Upper Immediate
    localparam logic [6:0] OPCODE_AUIPC  = 7'b0010111;  // Add Upper Immediate to PC
    localparam logic [6:0] OPCODE_JAL    = 7'b1101111;  // Jump And Link
    localparam logic [6:0] OPCODE_JALR   = 7'b1100111;  // Jump And Link Register
    localparam logic [6:0] OPCODE_BRANCH = 7'b1100011;  // 条件分岐（BEQ/BNE/BLT/BGE/BLTU/BGEU）
    localparam logic [6:0] OPCODE_LOAD   = 7'b0000011;  // ロード命令（LB/LH/LW/LBU/LHU）
    localparam logic [6:0] OPCODE_STORE  = 7'b0100011;  // ストア命令（SB/SH/SW）
    localparam logic [6:0] OPCODE_OPIMM  = 7'b0010011;  // 即値演算命令（ADDI/SLTI等）
    localparam logic [6:0] OPCODE_OP     = 7'b0110011;  // レジスタ間演算命令（ADD/SUB等）
    localparam logic [6:0] OPCODE_FENCE  = 7'b0001111;  // メモリ順序付け命令（FENCE/FENCE.I）
    localparam logic [6:0] OPCODE_SYSTEM = 7'b1110011;  // システム命令（ECALL/EBREAK等）
    localparam logic [31:0] RESET_INSTR_NOP = 32'h0000_0013;  // リセット時の初期命令（ADDI x0,x0,0 = NOP）

    // -----------------------------------------------------------------------
    // ステートマシン定義（多サイクル実行の各フェーズ）
    // -----------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATE_FETCH,      // 命令フェッチ：PCのアドレスから命令を読み出す
        STATE_DECODE,     // デコード：命令フィールドを解析してオペランドを準備する
        STATE_EXECUTE,    // 実行：ALU演算・分岐判定・アドレス計算を行う
        STATE_MEMORY,     // メモリアクセス：データメモリの読み書きを行う
        STATE_WRITEBACK,  // ライトバック：演算結果をレジスタファイルに書き戻す
        STATE_TRAP        // トラップ：例外発生時に遷移し、以降この状態に留まる
    } state_t;

    // -----------------------------------------------------------------------
    // レジスタ（フリップフロップ）：_q サフィックスは順序素子を示す
    // -----------------------------------------------------------------------
    state_t state_q;        // 現在のステートマシン状態

    logic [31:0] pc_q;      // プログラムカウンタ
    logic [31:0] instr_q;   // フェッチした命令レジスタ
    logic [31:0] next_pc_q; // 次のPC（ライトバック時にpc_qへ書き込む）
    logic [31:0] wb_data_q; // ライトバックデータ
    logic [4:0]  wb_rd_q;   // ライトバック先レジスタインデックス
    logic        wb_we_q;   // ライトバック書き込みイネーブル
    logic [31:0] regs_q [0:31];  // 汎用レジスタファイル（x0〜x31、x0は常に0）

    // -----------------------------------------------------------------------
    // 組み合わせ論理信号：_d サフィックスはデコード・実行段の組み合わせ値を示す
    // -----------------------------------------------------------------------
    logic [6:0]  opcode_d;               // 命令のオペコードフィールド[6:0]
    logic [2:0]  funct3_d;               // 命令のfunct3フィールド[14:12]
    logic [6:0]  funct7_d;               // 命令のfunct7フィールド[31:25]
    logic [4:0]  rs1_idx_d;              // ソースレジスタ1インデックス
    logic [4:0]  rs2_idx_d;              // ソースレジスタ2インデックス
    logic [4:0]  rd_idx_d;               // デスティネーションレジスタインデックス
    logic [31:0] rs1_val_d;              // ソースレジスタ1の値（x0は強制的に0）
    logic [31:0] rs2_val_d;              // ソースレジスタ2の値（x0は強制的に0）
    logic [31:0] imm_i_d;                // I形式即値（符号拡張）
    logic [31:0] imm_s_d;                // S形式即値（符号拡張）
    logic [31:0] imm_b_d;                // B形式即値（符号拡張、最下位ビットは0）
    logic [31:0] imm_u_d;                // U形式即値（上位20ビット、下位12ビットは0）
    logic [31:0] imm_j_d;                // J形式即値（符号拡張、最下位ビットは0）
    logic [31:0] pc_plus_4_d;            // PC+4（次の順次命令アドレス）
    logic [31:0] alu_result_d;           // ALU演算結果
    logic [31:0] mem_addr_d;             // メモリアクセスアドレス
    logic [31:0] branch_target_d;        // 分岐・ジャンプ先アドレス
    logic [31:0] jalr_target_sum_d;      // JALR計算中間値（rs1 + imm_i）
    logic [31:0] jalr_target_aligned_d;  // JALRジャンプ先（最下位ビットをクリアしてアライン）
    logic [31:0] load_data_d;            // ロード命令の読み出しデータ（サイズ・符号拡張後）
    logic [31:0] store_wdata_d;          // ストア命令の書き込みデータ（バイト位置に配置済み）
    logic [3:0]  store_wstrb_d;          // ストア命令の書き込みストローブ（バイトイネーブル）
    logic        branch_taken_d;         // 条件分岐が成立したか否か
    logic        exec_illegal_d;         // 実行ステージでの不正命令検出
    logic        mem_illegal_d;          // メモリアクセスのアライメントエラー検出
    logic        branch_misaligned_d;    // 分岐先アドレスのアライメントエラー検出
    logic        dmem_access_active_d;   // データメモリアクセス有効（LOAD/STOREかつ正常）
    logic        dmem_store_in_mem_state_d; // MEMORYステートでのストア操作中フラグ
    logic        dmem_store_active_d;    // ストアデータ出力有効フラグ
    integer      reg_idx;                // レジスタ初期化ループカウンタ

    // -----------------------------------------------------------------------
    // ヘルパー関数：バイトデータを32ビットワード内の指定バイト位置に配置する
    // byte_offset=0: [7:0], 1: [15:8], 2: [23:16], 3: [31:24]
    // -----------------------------------------------------------------------
    function automatic logic [31:0] make_byte_word(
        input logic [7:0] data_byte,
        input logic [1:0] byte_offset
    );
        logic [31:0] tmp;
        tmp = 32'h0000_0000;
        case (byte_offset)
            2'd0: tmp[7:0]   = data_byte;
            2'd1: tmp[15:8]  = data_byte;
            2'd2: tmp[23:16] = data_byte;
            2'd3: tmp[31:24] = data_byte;
            default: tmp = 32'h0000_0000;
        endcase
        make_byte_word = tmp;
    endfunction

    // -----------------------------------------------------------------------
    // ヘルパー関数：16ビットハーフワードを32ビットワードの上位または下位半分に配置する
    // upper_half=0: [15:0]に配置, upper_half=1: [31:16]に配置
    // -----------------------------------------------------------------------
    function automatic logic [31:0] make_half_word(
        input logic [15:0] data_half,
        input logic        upper_half
    );
        if (upper_half) begin
            make_half_word = {data_half, 16'h0000};
        end else begin
            make_half_word = {16'h0000, data_half};
        end
    endfunction

    // -----------------------------------------------------------------------
    // 組み合わせ論理ブロック：命令デコードおよび実行結果の計算
    // フェッチ済み命令(instr_q)をデコードし、ALU演算・分岐判定・メモリアドレスを計算する。
    // ステート非依存で常に評価されるが、有効な値はステートマシン側で選択して使用する。
    // -----------------------------------------------------------------------
    always_comb begin
        // ---- 命令フィールドの分解 ----
        opcode_d   = instr_q[6:0];
        funct3_d   = instr_q[14:12];
        funct7_d   = instr_q[31:25];
        rs1_idx_d  = instr_q[19:15];
        rs2_idx_d  = instr_q[24:20];
        rd_idx_d   = instr_q[11:7];
        // x0は常に0（レジスタファイルの値を無視）
        rs1_val_d  = (rs1_idx_d == 5'd0) ? 32'h0000_0000 : regs_q[rs1_idx_d];
        rs2_val_d  = (rs2_idx_d == 5'd0) ? 32'h0000_0000 : regs_q[rs2_idx_d];
        // 各命令形式の即値を符号拡張して生成
        imm_i_d    = {{20{instr_q[31]}}, instr_q[31:20]};
        imm_s_d    = {{20{instr_q[31]}}, instr_q[31:25], instr_q[11:7]};
        imm_b_d    = {{19{instr_q[31]}}, instr_q[31], instr_q[7], instr_q[30:25], instr_q[11:8], 1'b0};
        imm_u_d    = {instr_q[31:12], 12'h000};
        imm_j_d    = {{11{instr_q[31]}}, instr_q[31], instr_q[19:12], instr_q[20], instr_q[30:21], 1'b0};
        pc_plus_4_d = pc_q + 32'd4;

        // ---- 組み合わせ論理信号のデフォルト値（後続のcaseで上書き） ----
        alu_result_d        = 32'h0000_0000;
        mem_addr_d          = 32'h0000_0000;
        branch_target_d     = 32'h0000_0000;
        jalr_target_sum_d   = 32'h0000_0000;
        jalr_target_aligned_d = 32'h0000_0000;
        load_data_d         = 32'h0000_0000;
        store_wdata_d       = 32'h0000_0000;
        store_wstrb_d       = 4'b0000;
        branch_taken_d      = 1'b0;
        exec_illegal_d      = 1'b0;
        mem_illegal_d       = 1'b0;
        branch_misaligned_d = 1'b0;

        // ---- オペコード別の演算・アドレス計算 ----
        case (opcode_d)
            OPCODE_LUI: begin
                // LUI: 上位20ビット即値をそのまま結果とする
                alu_result_d = imm_u_d;
            end

            OPCODE_AUIPC: begin
                // AUIPC: PCに上位20ビット即値を加算
                alu_result_d = pc_q + imm_u_d;
            end

            OPCODE_JAL: begin
                // JAL: PC相対ジャンプ先アドレスを計算
                branch_target_d     = pc_q + imm_j_d;
                branch_misaligned_d = |branch_target_d[1:0];
            end

            OPCODE_JALR: begin
                // JALR: rs1+imm の最下位ビットをクリアしてジャンプ先とする
                if (funct3_d != 3'b000) begin
                    exec_illegal_d = 1'b1;
                end else begin
                    jalr_target_sum_d     = rs1_val_d + imm_i_d;
                    jalr_target_aligned_d = jalr_target_sum_d & 32'hffff_fffe;
                    branch_target_d       = jalr_target_aligned_d;
                    branch_misaligned_d   = |jalr_target_aligned_d[1:0];
                end
            end

            OPCODE_BRANCH: begin
                // 条件分岐：funct3で比較種別を選択、成立時のみ分岐ターゲットへ
                branch_target_d = pc_q + imm_b_d;
                case (funct3_d)
                    3'b000: branch_taken_d = (rs1_val_d == rs2_val_d);              // BEQ
                    3'b001: branch_taken_d = (rs1_val_d != rs2_val_d);              // BNE
                    3'b100: branch_taken_d = ($signed(rs1_val_d) <  $signed(rs2_val_d));  // BLT
                    3'b101: branch_taken_d = ($signed(rs1_val_d) >= $signed(rs2_val_d));  // BGE
                    3'b110: branch_taken_d = (rs1_val_d < rs2_val_d);              // BLTU
                    3'b111: branch_taken_d = (rs1_val_d >= rs2_val_d);             // BGEU
                    default: exec_illegal_d = 1'b1;
                endcase
                branch_misaligned_d = branch_taken_d && |branch_target_d[1:0];
            end

            OPCODE_LOAD: begin
                // ロード命令：メモリアドレスを計算し、funct3でロードサイズと符号拡張を選択
                mem_addr_d = rs1_val_d + imm_i_d;
                case (funct3_d)
                    3'b000: begin  // LB: バイトロード（符号拡張）
                        case (mem_addr_d[1:0])
                            2'd0: load_data_d = {{24{dmem_rdata[7]}}, dmem_rdata[7:0]};
                            2'd1: load_data_d = {{24{dmem_rdata[15]}}, dmem_rdata[15:8]};
                            2'd2: load_data_d = {{24{dmem_rdata[23]}}, dmem_rdata[23:16]};
                            2'd3: load_data_d = {{24{dmem_rdata[31]}}, dmem_rdata[31:24]};
                            default: load_data_d = 32'h0000_0000;
                        endcase
                    end
                    3'b001: begin  // LH: ハーフワードロード（符号拡張）、1バイトアライン違反はtrap
                        if (mem_addr_d[0]) begin
                            mem_illegal_d = 1'b1;
                        end else if (mem_addr_d[1]) begin
                            load_data_d = {{16{dmem_rdata[31]}}, dmem_rdata[31:16]};
                        end else begin
                            load_data_d = {{16{dmem_rdata[15]}}, dmem_rdata[15:0]};
                        end
                    end
                    3'b010: begin  // LW: ワードロード、4バイトアライン必須
                        if (|mem_addr_d[1:0]) begin
                            mem_illegal_d = 1'b1;
                        end else begin
                            load_data_d = dmem_rdata;
                        end
                    end
                    3'b100: begin  // LBU: バイトロード（ゼロ拡張）
                        case (mem_addr_d[1:0])
                            2'd0: load_data_d = {24'h000000, dmem_rdata[7:0]};
                            2'd1: load_data_d = {24'h000000, dmem_rdata[15:8]};
                            2'd2: load_data_d = {24'h000000, dmem_rdata[23:16]};
                            2'd3: load_data_d = {24'h000000, dmem_rdata[31:24]};
                            default: load_data_d = 32'h0000_0000;
                        endcase
                    end
                    3'b101: begin  // LHU: ハーフワードロード（ゼロ拡張）、1バイトアライン違反はtrap
                        if (mem_addr_d[0]) begin
                            mem_illegal_d = 1'b1;
                        end else if (mem_addr_d[1]) begin
                            load_data_d = {16'h0000, dmem_rdata[31:16]};
                        end else begin
                            load_data_d = {16'h0000, dmem_rdata[15:0]};
                        end
                    end
                    default: begin
                        mem_illegal_d = 1'b1;
                    end
                endcase
            end

            OPCODE_STORE: begin
                // ストア命令：メモリアドレスを計算し、funct3でストアサイズを選択
                mem_addr_d = rs1_val_d + imm_s_d;
                case (funct3_d)
                    3'b000: begin  // SB: バイトストア（アライン不要）
                        store_wstrb_d = 4'b0001 << mem_addr_d[1:0];
                        store_wdata_d = make_byte_word(rs2_val_d[7:0], mem_addr_d[1:0]);
                    end
                    3'b001: begin  // SH: ハーフワードストア（2バイトアライン必須）
                        if (mem_addr_d[0]) begin
                            mem_illegal_d = 1'b1;
                        end else begin
                            store_wstrb_d = mem_addr_d[1] ? 4'b1100 : 4'b0011;
                            store_wdata_d = make_half_word(rs2_val_d[15:0], mem_addr_d[1]);
                        end
                    end
                    3'b010: begin  // SW: ワードストア（4バイトアライン必須）
                        if (|mem_addr_d[1:0]) begin
                            mem_illegal_d = 1'b1;
                        end else begin
                            store_wstrb_d = 4'b1111;
                            store_wdata_d = rs2_val_d;
                        end
                    end
                    default: begin
                        mem_illegal_d = 1'b1;
                    end
                endcase
            end

            OPCODE_OPIMM: begin
                // 即値演算命令：funct3で演算種別を選択（シフト量はinstr_q[24:20]）
                case (funct3_d)
                    3'b000: alu_result_d = rs1_val_d + imm_i_d;                                    // ADDI
                    3'b010: alu_result_d = ($signed(rs1_val_d) < $signed(imm_i_d)) ? 32'd1 : 32'd0; // SLTI
                    3'b011: alu_result_d = (rs1_val_d < imm_i_d) ? 32'd1 : 32'd0;                 // SLTIU
                    3'b100: alu_result_d = rs1_val_d ^ imm_i_d;                                    // XORI
                    3'b110: alu_result_d = rs1_val_d | imm_i_d;                                    // ORI
                    3'b111: alu_result_d = rs1_val_d & imm_i_d;                                    // ANDI
                    3'b001: begin  // SLLI: 左論理シフト（funct7=0x00のみ有効）
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d << instr_q[24:20];
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b101: begin  // SRLI/SRAI: 右シフト（funct7で論理/算術を選択）
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d >> instr_q[24:20];          // SRLI: 右論理シフト
                        end else if (funct7_d == 7'b0100000) begin
                            alu_result_d = $signed(rs1_val_d) >>> instr_q[24:20]; // SRAI: 右算術シフト
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    default: exec_illegal_d = 1'b1;
                endcase
            end

            OPCODE_OP: begin
                // レジスタ間演算命令：funct3とfunct7の組み合わせで演算種別を選択
                case (funct3_d)
                    3'b000: begin  // ADD/SUB（funct7で加算/減算を選択）
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d + rs2_val_d;  // ADD
                        end else if (funct7_d == 7'b0100000) begin
                            alu_result_d = rs1_val_d - rs2_val_d;  // SUB
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b001: begin  // SLL: 左論理シフト
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d << rs2_val_d[4:0];
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b010: begin  // SLT: 符号付き比較（rs1 < rs2 なら1）
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = ($signed(rs1_val_d) < $signed(rs2_val_d)) ? 32'd1 : 32'd0;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b011: begin  // SLTU: 符号なし比較（rs1 < rs2 なら1）
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = (rs1_val_d < rs2_val_d) ? 32'd1 : 32'd0;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b100: begin  // XOR
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d ^ rs2_val_d;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b101: begin  // SRL/SRA: 右シフト（funct7で論理/算術を選択）
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d >> rs2_val_d[4:0];          // SRL: 右論理シフト
                        end else if (funct7_d == 7'b0100000) begin
                            alu_result_d = $signed(rs1_val_d) >>> rs2_val_d[4:0]; // SRA: 右算術シフト
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b110: begin  // OR
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d | rs2_val_d;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b111: begin  // AND
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d & rs2_val_d;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    default: exec_illegal_d = 1'b1;
                endcase
            end

            OPCODE_FENCE: begin
                // FENCE: メモリ順序付け命令（この実装ではNOPとして扱う）
                // funct3=000 は FENCE（正常）、funct3=001 は FENCE.I（制約確認）
                if (funct3_d == 3'b000) begin
                    exec_illegal_d = 1'b0;
                end else if (funct3_d == 3'b001) begin
                    // FENCE.I: pred/succ/rs1/rd がすべて0であることを確認
                    exec_illegal_d = (instr_q[31:20] != 12'h000) || (rs1_idx_d != 5'd0) || (rd_idx_d != 5'd0);
                end else begin
                    exec_illegal_d = 1'b1;
                end
            end

            OPCODE_SYSTEM: begin
                // SYSTEM命令（ECALL/EBREAK等）：本実装では全てtrap扱い
                exec_illegal_d = 1'b1;
            end

            default: begin
                // 未定義オペコード：trap
                exec_illegal_d = 1'b1;
            end
        endcase
    end

    // -----------------------------------------------------------------------
    // 組み合わせ論理ブロック：外部ポートへの出力信号の生成
    // -----------------------------------------------------------------------
    always_comb begin
        // MEMORYステートでLOAD/STORE命令かつアライメント正常な場合のみデータメモリアクセス有効
        dmem_access_active_d = (state_q == STATE_MEMORY) &&
                               ((opcode_d == OPCODE_LOAD) || (opcode_d == OPCODE_STORE)) &&
                               !mem_illegal_d;
        dmem_store_in_mem_state_d = (state_q == STATE_MEMORY) && (opcode_d == OPCODE_STORE);
        dmem_store_active_d       = dmem_store_in_mem_state_d && !mem_illegal_d;
        imem_addr  = pc_q;                                                             // 命令フェッチアドレスは常にPC
        dmem_valid = dmem_access_active_d;
        dmem_addr  = mem_addr_d;
        dmem_wdata = dmem_store_active_d ? store_wdata_d : 32'h0000_0000;  // ストア時のみ書き込みデータを出力
        dmem_wstrb = dmem_store_active_d ? store_wstrb_d : 4'b0000;
        trap       = (state_q == STATE_TRAP);
    end

    // -----------------------------------------------------------------------
    // 順序回路ブロック：ステートマシンおよびレジスタの更新（クロック同期、非同期リセット）
    // -----------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // リセット時：全レジスタを初期値に設定
            state_q   <= STATE_FETCH;
            pc_q      <= RESET_PC;
            instr_q   <= RESET_INSTR_NOP;
            next_pc_q <= RESET_PC;
            wb_data_q <= 32'h0000_0000;
            wb_rd_q   <= 5'd0;
            wb_we_q   <= 1'b0;

            for (reg_idx = 0; reg_idx < 32; reg_idx = reg_idx + 1) begin
                regs_q[reg_idx] <= 32'h0000_0000;
            end
        end else begin
            case (state_q)
                STATE_FETCH: begin
                    // PCのアライメント確認（最下位2ビットが0でない場合はtrap）
                    if (|pc_q[1:0]) begin
                        state_q <= STATE_TRAP;
                    end else begin
                        instr_q <= imem_rdata;  // 命令メモリから命令を取り込む
                        state_q <= STATE_DECODE;
                    end
                end

                STATE_DECODE: begin
                    // デコードステートでは組み合わせ論理が命令を解析するのでDFFは次ステートへ遷移するだけ
                    state_q <= STATE_EXECUTE;
                end

                STATE_EXECUTE: begin
                    wb_we_q <= 1'b0;  // デフォルトはライトバック無効

                    if (exec_illegal_d || branch_misaligned_d) begin
                        // 不正命令またはミスアライン分岐はtrap
                        state_q <= STATE_TRAP;
                    end else begin
                        case (opcode_d)
                            OPCODE_LUI,
                            OPCODE_AUIPC,
                            OPCODE_OPIMM,
                            OPCODE_OP: begin
                                // ALU演算結果をライトバック用レジスタに保存してWRITEBACKへ
                                wb_data_q <= alu_result_d;
                                wb_rd_q   <= rd_idx_d;
                                wb_we_q   <= (rd_idx_d != 5'd0);  // rd=x0への書き込みは無視
                                next_pc_q <= pc_plus_4_d;
                                state_q   <= STATE_WRITEBACK;
                            end

                            OPCODE_JAL,
                            OPCODE_JALR: begin
                                // リンクアドレス（PC+4）をrdに書き込み、ジャンプ先PCを設定
                                wb_data_q <= pc_plus_4_d;
                                wb_rd_q   <= rd_idx_d;
                                wb_we_q   <= (rd_idx_d != 5'd0);
                                next_pc_q <= branch_target_d;
                                state_q   <= STATE_WRITEBACK;
                            end

                            OPCODE_BRANCH: begin
                                // 分岐成立時はbranch_target_d、不成立時はPC+4へ直接遷移
                                pc_q    <= branch_taken_d ? branch_target_d : pc_plus_4_d;
                                state_q <= STATE_FETCH;
                            end

                            OPCODE_LOAD,
                            OPCODE_STORE: begin
                                // アライメントエラーが既に検出されていればtrap、それ以外はMEMORYへ
                                if (mem_illegal_d) begin
                                    state_q <= STATE_TRAP;
                                end else begin
                                    state_q <= STATE_MEMORY;
                                end
                            end

                            OPCODE_FENCE: begin
                                // FENCEはNOPとして扱い、PC+4へ進む
                                pc_q    <= pc_plus_4_d;
                                state_q <= STATE_FETCH;
                            end

                            default: begin
                                state_q <= STATE_TRAP;
                            end
                        endcase
                    end
                end

                STATE_MEMORY: begin
                    if (mem_illegal_d) begin
                        state_q <= STATE_TRAP;
                    end else if (opcode_d == OPCODE_LOAD) begin
                        // ロード：メモリ読み出しデータをライトバック用レジスタに保存してWRITEBACKへ
                        wb_data_q <= load_data_d;
                        wb_rd_q   <= rd_idx_d;
                        wb_we_q   <= (rd_idx_d != 5'd0);
                        next_pc_q <= pc_plus_4_d;
                        state_q   <= STATE_WRITEBACK;
                    end else begin
                        pc_q    <= pc_plus_4_d;
                        state_q <= STATE_FETCH;
                    end
                end

                STATE_WRITEBACK: begin
                    // ライトバック：wb_we_qが有効な場合のみレジスタに書き込む
                    if (wb_we_q) begin
                        regs_q[wb_rd_q] <= wb_data_q;
                    end
                    pc_q    <= next_pc_q;  // 次のPCに更新してFETCHへ戻る
                    state_q <= STATE_FETCH;
                end

                STATE_TRAP: begin
                    // トラップ：このステートに留まり続ける（trap出力をアサート）
                    state_q <= STATE_TRAP;
                end

                default: begin
                    // 未定義ステート：安全のためtrapへ遷移
                    state_q <= STATE_TRAP;
                end
            endcase

            regs_q[0] <= 32'h0000_0000;  // x0は常に0（ライトバック後も強制クリア）
        end
    end

endmodule
