module rv32i_core #(
    parameter logic [31:0] RESET_PC = 32'h0000_0000
) (
    input  logic        clk,
    input  logic        rst_n,
    output logic [31:0] imem_addr,
    input  logic [31:0] imem_rdata,
    output logic        dmem_valid,
    output logic [31:0] dmem_addr,
    output logic [31:0] dmem_wdata,
    output logic [3:0]  dmem_wstrb,
    input  logic [31:0] dmem_rdata,
    output logic        trap
);

    localparam logic [6:0] OPCODE_LUI    = 7'b0110111;
    localparam logic [6:0] OPCODE_AUIPC  = 7'b0010111;
    localparam logic [6:0] OPCODE_JAL    = 7'b1101111;
    localparam logic [6:0] OPCODE_JALR   = 7'b1100111;
    localparam logic [6:0] OPCODE_BRANCH = 7'b1100011;
    localparam logic [6:0] OPCODE_LOAD   = 7'b0000011;
    localparam logic [6:0] OPCODE_STORE  = 7'b0100011;
    localparam logic [6:0] OPCODE_OPIMM  = 7'b0010011;
    localparam logic [6:0] OPCODE_OP     = 7'b0110011;
    localparam logic [6:0] OPCODE_FENCE  = 7'b0001111;
    localparam logic [6:0] OPCODE_SYSTEM = 7'b1110011;

    typedef enum logic [2:0] {
        STATE_FETCH,
        STATE_DECODE,
        STATE_EXECUTE,
        STATE_MEMORY,
        STATE_WRITEBACK,
        STATE_TRAP
    } state_t;

    state_t state_q;

    logic [31:0] pc_q;
    logic [31:0] instr_q;
    logic [31:0] next_pc_q;
    logic [31:0] wb_data_q;
    logic [4:0]  wb_rd_q;
    logic        wb_we_q;
    logic [31:0] regs_q [0:31];

    logic [6:0]  opcode_d;
    logic [2:0]  funct3_d;
    logic [6:0]  funct7_d;
    logic [4:0]  rs1_idx_d;
    logic [4:0]  rs2_idx_d;
    logic [4:0]  rd_idx_d;
    logic [31:0] rs1_val_d;
    logic [31:0] rs2_val_d;
    logic [31:0] imm_i_d;
    logic [31:0] imm_s_d;
    logic [31:0] imm_b_d;
    logic [31:0] imm_u_d;
    logic [31:0] imm_j_d;
    logic [31:0] pc_plus_4_d;
    logic [31:0] alu_result_d;
    logic [31:0] mem_addr_d;
    logic [31:0] branch_target_d;
    logic [31:0] jalr_target_raw_d;
    logic [31:0] load_data_d;
    logic [31:0] store_wdata_d;
    logic [3:0]  store_wstrb_d;
    logic        branch_taken_d;
    logic        exec_illegal_d;
    logic        mem_illegal_d;
    logic        branch_misaligned_d;
    logic        dmem_access_active_d;
    logic        dmem_store_active_d;
    integer      i;

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

    always_comb begin
        opcode_d   = instr_q[6:0];
        funct3_d   = instr_q[14:12];
        funct7_d   = instr_q[31:25];
        rs1_idx_d  = instr_q[19:15];
        rs2_idx_d  = instr_q[24:20];
        rd_idx_d   = instr_q[11:7];
        rs1_val_d  = (rs1_idx_d == 5'd0) ? 32'h0000_0000 : regs_q[rs1_idx_d];
        rs2_val_d  = (rs2_idx_d == 5'd0) ? 32'h0000_0000 : regs_q[rs2_idx_d];
        imm_i_d    = {{20{instr_q[31]}}, instr_q[31:20]};
        imm_s_d    = {{20{instr_q[31]}}, instr_q[31:25], instr_q[11:7]};
        imm_b_d    = {{19{instr_q[31]}}, instr_q[31], instr_q[7], instr_q[30:25], instr_q[11:8], 1'b0};
        imm_u_d    = {instr_q[31:12], 12'h000};
        imm_j_d    = {{11{instr_q[31]}}, instr_q[31], instr_q[19:12], instr_q[20], instr_q[30:21], 1'b0};
        pc_plus_4_d = pc_q + 32'd4;

        alu_result_d        = 32'h0000_0000;
        mem_addr_d          = 32'h0000_0000;
        branch_target_d     = 32'h0000_0000;
        jalr_target_raw_d   = 32'h0000_0000;
        load_data_d         = 32'h0000_0000;
        store_wdata_d       = 32'h0000_0000;
        store_wstrb_d       = 4'b0000;
        branch_taken_d      = 1'b0;
        exec_illegal_d      = 1'b0;
        mem_illegal_d       = 1'b0;
        branch_misaligned_d = 1'b0;

        case (opcode_d)
            OPCODE_LUI: begin
                alu_result_d = imm_u_d;
            end

            OPCODE_AUIPC: begin
                alu_result_d = pc_q + imm_u_d;
            end

            OPCODE_JAL: begin
                branch_target_d     = pc_q + imm_j_d;
                branch_misaligned_d = |branch_target_d[1:0];
            end

            OPCODE_JALR: begin
                if (funct3_d != 3'b000) begin
                    exec_illegal_d = 1'b1;
                end else begin
                    jalr_target_raw_d   = rs1_val_d + imm_i_d;
                    branch_target_d     = jalr_target_raw_d & 32'hffff_fffe;
                    branch_misaligned_d = jalr_target_raw_d[1];
                end
            end

            OPCODE_BRANCH: begin
                branch_target_d = pc_q + imm_b_d;
                case (funct3_d)
                    3'b000: branch_taken_d = (rs1_val_d == rs2_val_d);
                    3'b001: branch_taken_d = (rs1_val_d != rs2_val_d);
                    3'b100: branch_taken_d = ($signed(rs1_val_d) <  $signed(rs2_val_d));
                    3'b101: branch_taken_d = ($signed(rs1_val_d) >= $signed(rs2_val_d));
                    3'b110: branch_taken_d = (rs1_val_d < rs2_val_d);
                    3'b111: branch_taken_d = (rs1_val_d >= rs2_val_d);
                    default: exec_illegal_d = 1'b1;
                endcase
                branch_misaligned_d = branch_taken_d && |branch_target_d[1:0];
            end

            OPCODE_LOAD: begin
                mem_addr_d = rs1_val_d + imm_i_d;
                case (funct3_d)
                    3'b000: begin
                        case (mem_addr_d[1:0])
                            2'd0: load_data_d = {{24{dmem_rdata[7]}}, dmem_rdata[7:0]};
                            2'd1: load_data_d = {{24{dmem_rdata[15]}}, dmem_rdata[15:8]};
                            2'd2: load_data_d = {{24{dmem_rdata[23]}}, dmem_rdata[23:16]};
                            2'd3: load_data_d = {{24{dmem_rdata[31]}}, dmem_rdata[31:24]};
                            default: load_data_d = 32'h0000_0000;
                        endcase
                    end
                    3'b001: begin
                        if (mem_addr_d[0]) begin
                            mem_illegal_d = 1'b1;
                        end else if (mem_addr_d[1]) begin
                            load_data_d = {{16{dmem_rdata[31]}}, dmem_rdata[31:16]};
                        end else begin
                            load_data_d = {{16{dmem_rdata[15]}}, dmem_rdata[15:0]};
                        end
                    end
                    3'b010: begin
                        if (|mem_addr_d[1:0]) begin
                            mem_illegal_d = 1'b1;
                        end else begin
                            load_data_d = dmem_rdata;
                        end
                    end
                    3'b100: begin
                        case (mem_addr_d[1:0])
                            2'd0: load_data_d = {24'h000000, dmem_rdata[7:0]};
                            2'd1: load_data_d = {24'h000000, dmem_rdata[15:8]};
                            2'd2: load_data_d = {24'h000000, dmem_rdata[23:16]};
                            2'd3: load_data_d = {24'h000000, dmem_rdata[31:24]};
                            default: load_data_d = 32'h0000_0000;
                        endcase
                    end
                    3'b101: begin
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
                mem_addr_d = rs1_val_d + imm_s_d;
                case (funct3_d)
                    3'b000: begin
                        store_wstrb_d = 4'b0001 << mem_addr_d[1:0];
                        store_wdata_d = make_byte_word(rs2_val_d[7:0], mem_addr_d[1:0]);
                    end
                    3'b001: begin
                        if (mem_addr_d[0]) begin
                            mem_illegal_d = 1'b1;
                        end else begin
                            store_wstrb_d = mem_addr_d[1] ? 4'b1100 : 4'b0011;
                            store_wdata_d = make_half_word(rs2_val_d[15:0], mem_addr_d[1]);
                        end
                    end
                    3'b010: begin
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
                case (funct3_d)
                    3'b000: alu_result_d = rs1_val_d + imm_i_d;
                    3'b010: alu_result_d = ($signed(rs1_val_d) < $signed(imm_i_d)) ? 32'd1 : 32'd0;
                    3'b011: alu_result_d = (rs1_val_d < imm_i_d) ? 32'd1 : 32'd0;
                    3'b100: alu_result_d = rs1_val_d ^ imm_i_d;
                    3'b110: alu_result_d = rs1_val_d | imm_i_d;
                    3'b111: alu_result_d = rs1_val_d & imm_i_d;
                    3'b001: begin
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d << instr_q[24:20];
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b101: begin
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d >> instr_q[24:20];
                        end else if (funct7_d == 7'b0100000) begin
                            alu_result_d = $signed(rs1_val_d) >>> instr_q[24:20];
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    default: exec_illegal_d = 1'b1;
                endcase
            end

            OPCODE_OP: begin
                case (funct3_d)
                    3'b000: begin
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d + rs2_val_d;
                        end else if (funct7_d == 7'b0100000) begin
                            alu_result_d = rs1_val_d - rs2_val_d;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b001: begin
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d << rs2_val_d[4:0];
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b010: begin
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = ($signed(rs1_val_d) < $signed(rs2_val_d)) ? 32'd1 : 32'd0;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b011: begin
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = (rs1_val_d < rs2_val_d) ? 32'd1 : 32'd0;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b100: begin
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d ^ rs2_val_d;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b101: begin
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d >> rs2_val_d[4:0];
                        end else if (funct7_d == 7'b0100000) begin
                            alu_result_d = $signed(rs1_val_d) >>> rs2_val_d[4:0];
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b110: begin
                        if (funct7_d == 7'b0000000) begin
                            alu_result_d = rs1_val_d | rs2_val_d;
                        end else begin
                            exec_illegal_d = 1'b1;
                        end
                    end
                    3'b111: begin
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
                if (funct3_d != 3'b000 && funct3_d != 3'b001) begin
                    exec_illegal_d = 1'b1;
                end
            end

            OPCODE_SYSTEM: begin
                exec_illegal_d = 1'b1;
            end

            default: begin
                exec_illegal_d = 1'b1;
            end
        endcase
    end

    always_comb begin
        dmem_access_active_d = (state_q == STATE_MEMORY) && ((opcode_d == OPCODE_LOAD) || (opcode_d == OPCODE_STORE));
        dmem_store_active_d  = (state_q == STATE_MEMORY) && (opcode_d == OPCODE_STORE);
        imem_addr  = pc_q;
        dmem_valid = dmem_access_active_d;
        dmem_addr  = mem_addr_d;
        dmem_wdata = dmem_store_active_d ? store_wdata_d : 32'h0000_0000;
        dmem_wstrb = dmem_store_active_d ? store_wstrb_d : 4'b0000;
        trap       = (state_q == STATE_TRAP);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q   <= STATE_FETCH;
            pc_q      <= RESET_PC;
            instr_q   <= 32'h0000_0013;
            next_pc_q <= RESET_PC;
            wb_data_q <= 32'h0000_0000;
            wb_rd_q   <= 5'd0;
            wb_we_q   <= 1'b0;

            for (i = 0; i < 32; i = i + 1) begin
                regs_q[i] <= 32'h0000_0000;
            end
        end else begin
            case (state_q)
                STATE_FETCH: begin
                    if (|pc_q[1:0]) begin
                        state_q <= STATE_TRAP;
                    end else begin
                        instr_q <= imem_rdata;
                        state_q <= STATE_DECODE;
                    end
                end

                STATE_DECODE: begin
                    state_q <= STATE_EXECUTE;
                end

                STATE_EXECUTE: begin
                    wb_we_q <= 1'b0;

                    if (exec_illegal_d || branch_misaligned_d) begin
                        state_q <= STATE_TRAP;
                    end else begin
                        case (opcode_d)
                            OPCODE_LUI,
                            OPCODE_AUIPC,
                            OPCODE_OPIMM,
                            OPCODE_OP: begin
                                wb_data_q <= alu_result_d;
                                wb_rd_q   <= rd_idx_d;
                                wb_we_q   <= (rd_idx_d != 5'd0);
                                next_pc_q <= pc_plus_4_d;
                                state_q   <= STATE_WRITEBACK;
                            end

                            OPCODE_JAL,
                            OPCODE_JALR: begin
                                wb_data_q <= pc_plus_4_d;
                                wb_rd_q   <= rd_idx_d;
                                wb_we_q   <= (rd_idx_d != 5'd0);
                                next_pc_q <= branch_target_d;
                                state_q   <= STATE_WRITEBACK;
                            end

                            OPCODE_BRANCH: begin
                                pc_q    <= branch_taken_d ? branch_target_d : pc_plus_4_d;
                                state_q <= STATE_FETCH;
                            end

                            OPCODE_LOAD,
                            OPCODE_STORE: begin
                                state_q <= STATE_MEMORY;
                            end

                            OPCODE_FENCE: begin
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
                    if (wb_we_q && (wb_rd_q != 5'd0)) begin
                        regs_q[wb_rd_q] <= wb_data_q;
                    end
                    pc_q    <= next_pc_q;
                    state_q <= STATE_FETCH;
                end

                STATE_TRAP: begin
                    state_q <= STATE_TRAP;
                end

                default: begin
                    state_q <= STATE_TRAP;
                end
            endcase

            regs_q[0] <= 32'h0000_0000;
        end
    end

endmodule
