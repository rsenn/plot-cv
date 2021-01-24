(module
  (type $t0 (func (param i32) (result i32)))
  (type $t1 (func (param i32)))
  (type $t2 (func (param i32 i32) (result i32)))
  (type $t3 (func (param i32 i32 i32) (result i32)))
  (type $t4 (func (param i32 i32)))
  (type $t5 (func (param i32 i32 i32)))
  (type $t6 (func (param i32 i32 i32 i32 i32 i32)))
  (type $t7 (func (param i32 i32 i32 i32)))
  (type $t8 (func (param i32 i32 i32 i32 i32)))
  (type $t9 (func (param i32 i32 i32 i32) (result i32)))
  (type $t10 (func (param i32 i32 i32 i32 i32 i32 i32)))
  (type $t11 (func))
  (type $t12 (func (result i32)))
  (type $t13 (func (param i32 i32 i32 i32 i32) (result i32)))
  (type $t14 (func (param i32 i32 i32 i32 i32 i32 i32 i32 i32 i32)))
  (type $t15 (func (param i32 i32 i32 i32 i32 i32 i32 i32 i32 i32 i32 i32)))
  (type $t16 (func (param i32 i32 i32 i32 i32 i32 i32 i32)))
  (type $t17 (func (param i32 i32 i32 i32 i32 i32 i32 i32 i32)))
  (type $t18 (func (param i32 i32 i32 i32 i32 i32) (result i32)))
  (type $t19 (func (param i32 i32 i32 i32 i32 i32 i32 i32) (result i32)))
  (type $t20 (func (param f64) (result i32)))
  (type $t21 (func (param i32 i32 i32 i32 i32 i32 i32 i32 i32) (result i32)))
  (type $t22 (func (param i32 i32 i32 i32 i32 i32 i32 i32 i32 i32 i32 i32) (result i32)))
  (type $t23 (func (param i32 i32 i32 i32 i32 i32 i32 i32 i32 i32 i32 i32 i32) (result i32)))
  (import "env" "abort" (func $abort (type $t11)))
  (import "env" "emscripten_resize_heap" (func $emscripten_resize_heap (type $t0)))
  (import "env" "emscripten_memcpy_big" (func $emscripten_memcpy_big (type $t3)))
  (func $__wasm_call_ctors (export "__wasm_call_ctors") (type $t11)
    nop)
  (func $ff_hevc_save_states (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32)
    block $B0
      get_local $p0
      i32.load offset=204
      i32.load8_u offset=43
      i32.eqz
      br_if $B0
      get_local $p1
      get_local $p0
      i32.load offset=200
      i32.load offset=13128
      tee_local $p1
      i32.rem_s
      tee_local $l0
      i32.const 2
      i32.ne
      i32.const 0
      get_local $p1
      i32.const 2
      i32.ne
      get_local $l0
      i32.or
      select
      br_if $B0
      get_local $p0
      i32.load offset=152
      get_local $p0
      i32.load offset=136
      i32.const 199
      call $memcpy
      drop
    end)
  (func $ff_hevc_cabac_init (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32) (local $l1 i32)
    block $B0
      block $B1
        block $B2
          get_local $p1
          get_local $p0
          i32.load offset=204
          tee_local $l0
          i32.load offset=1668
          get_local $p0
          i32.const 2500
          i32.add
          i32.load
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.eq
          if $I3
            get_local $p0
            call $cabac_init_decoder
            block $B4
              get_local $p0
              i32.const 1449
              i32.add
              i32.load8_u
              if $I5
                get_local $p0
                i32.load offset=204
                tee_local $l0
                i32.load8_u offset=42
                i32.eqz
                br_if $B4
                get_local $l0
                i32.load offset=1676
                get_local $p1
                i32.const 2
                i32.shl
                i32.add
                tee_local $l0
                i32.load
                get_local $l0
                i32.const 4
                i32.sub
                i32.load
                i32.eq
                br_if $B4
              end
              get_local $p0
              call $cabac_init_state
            end
            get_local $p0
            i32.const 1448
            i32.add
            i32.load8_u
            br_if $B1
            get_local $p0
            i32.load offset=204
            i32.load8_u offset=43
            i32.eqz
            br_if $B1
            get_local $p1
            get_local $p0
            i32.load offset=200
            i32.load offset=13128
            tee_local $p1
            i32.rem_s
            br_if $B1
            get_local $p1
            i32.const 1
            i32.eq
            br_if $B0
            get_local $p0
            i32.load8_u offset=1449
            i32.const 1
            i32.eq
            br_if $B2
            br $B1
          end
          block $B6
            get_local $l0
            i32.load8_u offset=42
            i32.eqz
            br_if $B6
            get_local $l0
            i32.load offset=1676
            get_local $p1
            i32.const 2
            i32.shl
            i32.add
            tee_local $l1
            i32.load
            get_local $l1
            i32.const 4
            i32.sub
            i32.load
            i32.eq
            br_if $B6
            block $B7
              get_local $p0
              i32.load8_u offset=141
              i32.const 1
              i32.eq
              if $I8
                get_local $p0
                i32.load offset=136
                call $cabac_reinit
                br $B7
              end
              get_local $p0
              call $cabac_init_decoder
            end
            get_local $p0
            call $cabac_init_state
            get_local $p0
            i32.load offset=204
            set_local $l0
          end
          get_local $l0
          i32.load8_u offset=43
          i32.eqz
          br_if $B1
          get_local $p1
          get_local $p0
          i32.load offset=200
          i32.load offset=13128
          i32.rem_s
          br_if $B1
          get_local $p0
          i32.load offset=136
          i32.const 224
          i32.add
          call $get_cabac_terminate
          drop
          block $B9
            get_local $p0
            i32.load8_u offset=141
            i32.const 1
            i32.eq
            if $I10
              get_local $p0
              i32.load offset=136
              call $cabac_reinit
              br $B9
            end
            get_local $p0
            call $cabac_init_decoder
          end
          get_local $p0
          i32.load offset=200
          i32.load offset=13128
          i32.const 1
          i32.eq
          br_if $B0
        end
        get_local $p0
        i32.load offset=136
        get_local $p0
        i32.load offset=152
        i32.const 199
        call $memcpy
        drop
      end
      return
    end
    get_local $p0
    call $cabac_init_state)
  (func $cabac_init_decoder (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p0
    i32.load offset=136
    tee_local $l1
    i32.const 204
    i32.add
    tee_local $l0
    i32.const 1
    call $skip_bits
    i32.const 0
    get_local $l0
    tee_local $l2
    i32.load offset=8
    i32.sub
    i32.const 7
    i32.and
    tee_local $l3
    if $I0
      get_local $l2
      get_local $l3
      call $skip_bits
    end
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    get_local $l1
    i32.load offset=204
    get_local $l0
    i32.load offset=8
    i32.const 8
    i32.div_s
    i32.add
    get_local $l0
    call $get_bits_left
    i32.const 7
    i32.add
    i32.const 8
    i32.div_s
    call $ff_init_cabac_decoder)
  (func $cabac_init_state (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    i32.const 2
    get_local $p0
    i32.const 1440
    i32.add
    i32.load
    tee_local $l0
    i32.sub
    tee_local $l1
    get_local $l1
    i32.const 3
    i32.xor
    get_local $l1
    get_local $p0
    i32.const 2060
    i32.add
    i32.load8_u
    select
    get_local $l0
    i32.const 2
    i32.eq
    select
    i32.const 199
    i32.mul
    set_local $l3
    i32.const 0
    set_local $l1
    loop $L0
      get_local $p0
      i32.load offset=136
      get_local $l1
      i32.add
      get_local $l1
      get_local $l3
      i32.add
      i32.const 1712
      i32.add
      i32.load8_u
      tee_local $l0
      i32.const 3
      i32.shl
      i32.const 120
      i32.and
      get_local $l0
      i32.const 4
      i32.shr_u
      i32.const 5
      i32.mul
      i32.const 45
      i32.sub
      get_local $p0
      i32.load8_s offset=2112
      tee_local $l0
      i32.const 51
      get_local $l0
      i32.const 51
      i32.lt_s
      select
      tee_local $l0
      i32.const 0
      get_local $l0
      i32.const 0
      i32.gt_s
      select
      i32.mul
      i32.const 4
      i32.shr_s
      i32.add
      i32.const 1
      i32.shl
      i32.const 159
      i32.sub
      tee_local $l0
      i32.const 31
      i32.shr_s
      get_local $l0
      i32.xor
      tee_local $l0
      i32.const 1
      i32.and
      i32.const 124
      i32.or
      get_local $l0
      get_local $l0
      i32.const 124
      i32.gt_s
      select
      i32.store8
      get_local $l1
      i32.const 1
      i32.add
      tee_local $l1
      i32.const 199
      i32.ne
      br_if $L0
    end
    loop $L1
      get_local $p0
      i32.load offset=136
      get_local $l2
      i32.add
      i32.const 0
      i32.store8 offset=199
      get_local $l2
      i32.const 1
      i32.add
      tee_local $l2
      i32.const 4
      i32.ne
      br_if $L1
    end)
  (func $cabac_reinit (type $t1) (param $p0 i32)
    get_local $p0
    i32.const 224
    i32.add
    call $skip_bytes)
  (func $get_cabac_terminate (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    get_local $p0
    i32.load offset=4
    i32.const 2
    i32.sub
    tee_local $l0
    i32.store offset=4
    get_local $p0
    i32.load
    get_local $l0
    i32.const 17
    i32.shl
    i32.lt_s
    if $I0
      get_local $p0
      call $renorm_cabac_decoder_once
      i32.const 0
      return
    end
    get_local $p0
    i32.load offset=16
    get_local $p0
    i32.load offset=12
    i32.sub)
  (func $get_bits_left (type $t0) (param $p0 i32) (result i32)
    get_local $p0
    i32.load offset=12
    get_local $p0
    i32.load offset=8
    i32.sub)
  (func $skip_bytes (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load offset=20
    get_local $p0
    i32.load offset=16
    tee_local $l0
    i32.const 1
    i32.sub
    get_local $l0
    get_local $p0
    i32.load
    tee_local $l0
    i32.const 1
    i32.and
    select
    tee_local $l1
    i32.const 1
    i32.sub
    get_local $l1
    get_local $l0
    i32.const 511
    i32.and
    select
    tee_local $l0
    i32.sub
    tee_local $l1
    i32.const 0
    i32.ge_s
    if $I0
      get_local $p0
      get_local $l0
      get_local $l1
      call $ff_init_cabac_decoder
    end)
  (func $renorm_cabac_decoder_once (type $t1) (param $p0 i32)
    (local $l0 i32)
    get_local $p0
    get_local $p0
    i32.load offset=4
    tee_local $l0
    get_local $l0
    i32.const 256
    i32.sub
    i32.const 31
    i32.shr_u
    tee_local $l0
    i32.shl
    i32.store offset=4
    get_local $p0
    get_local $p0
    i32.load
    get_local $l0
    i32.shl
    tee_local $l0
    i32.store
    get_local $l0
    i32.const 65535
    i32.and
    i32.eqz
    if $I0
      get_local $p0
      call $refill
    end)
  (func $ff_hevc_sao_merge_flag_decode (type $t0) (param $p0 i32) (result i32)
    get_local $p0
    i32.load offset=136
    tee_local $p0
    i32.const 224
    i32.add
    get_local $p0
    call $get_cabac)
  (func $get_cabac (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32)
    get_local $p0
    get_local $p1
    i32.load8_u
    tee_local $l2
    get_local $p0
    i32.load offset=4
    tee_local $l0
    i32.const 1
    i32.shl
    i32.const 384
    i32.and
    i32.add
    i32.const 5296
    i32.add
    i32.load8_u
    tee_local $l1
    get_local $l0
    get_local $l1
    i32.sub
    tee_local $l0
    i32.sub
    i32.const -1
    i32.const 0
    get_local $l0
    i32.const 17
    i32.shl
    tee_local $l3
    get_local $p0
    i32.load
    tee_local $l4
    i32.lt_s
    select
    tee_local $l1
    i32.and
    get_local $l0
    i32.add
    i32.store offset=4
    get_local $p0
    get_local $l4
    get_local $l1
    get_local $l3
    i32.and
    i32.sub
    i32.store
    get_local $p1
    get_local $l1
    get_local $l2
    i32.xor
    tee_local $p1
    i32.const 5936
    i32.add
    i32.load8_u
    i32.store8
    get_local $p0
    get_local $p0
    i32.load offset=4
    tee_local $l0
    get_local $l0
    i32.const 4784
    i32.add
    i32.load8_u
    tee_local $l0
    i32.shl
    i32.store offset=4
    get_local $p0
    get_local $p0
    i32.load
    get_local $l0
    i32.shl
    tee_local $l0
    i32.store
    get_local $l0
    i32.const 65535
    i32.and
    i32.eqz
    if $I0
      get_local $p0
      call $refill2
    end
    get_local $p1
    i32.const 1
    i32.and)
  (func $refill2 (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    get_local $p0
    i32.load offset=16
    tee_local $l0
    i32.load8_u offset=1
    i32.const 1
    i32.shl
    get_local $l0
    i32.load8_u
    i32.const 9
    i32.shl
    i32.or
    i32.const 65535
    i32.sub
    i32.const 7
    get_local $p0
    i32.load
    tee_local $l1
    i32.const 1
    i32.sub
    get_local $l1
    i32.xor
    i32.const 15
    i32.shr_s
    i32.const 4784
    i32.add
    i32.load8_u
    i32.sub
    i32.shl
    get_local $l1
    i32.add
    i32.store
    get_local $p0
    i32.load offset=20
    get_local $l0
    i32.gt_u
    if $I0
      get_local $p0
      get_local $l0
      i32.const 2
      i32.add
      i32.store offset=16
    end)
  (func $ff_hevc_sao_type_idx_decode (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    i32.load offset=136
    tee_local $l0
    i32.const 224
    i32.add
    get_local $l0
    i32.const 1
    i32.add
    call $get_cabac
    if $I0
      i32.const 2
      i32.const 1
      get_local $p0
      i32.load offset=136
      i32.const 224
      i32.add
      call $get_cabac_bypass
      select
      return
    end
    i32.const 0)
  (func $get_cabac_bypass (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_local $p0
    get_local $p0
    i32.load
    tee_local $l1
    i32.const 1
    i32.shl
    tee_local $l0
    i32.store
    get_local $l1
    i32.const 32767
    i32.and
    i32.eqz
    if $I0
      get_local $p0
      call $refill
      get_local $p0
      i32.load
      set_local $l0
    end
    i32.const 0
    set_local $l1
    get_local $p0
    i32.load offset=4
    i32.const 17
    i32.shl
    tee_local $l2
    get_local $l0
    i32.le_s
    if $I1 (result i32)
      get_local $p0
      get_local $l0
      get_local $l2
      i32.sub
      i32.store
      i32.const 1
    else
      get_local $l1
    end)
  (func $refill (type $t1) (param $p0 i32)
    (local $l0 i32)
    get_local $p0
    get_local $p0
    i32.load
    get_local $p0
    i32.load offset=16
    tee_local $l0
    i32.load8_u offset=1
    i32.const 1
    i32.shl
    get_local $l0
    i32.load8_u
    i32.const 9
    i32.shl
    i32.or
    i32.add
    i32.const 65535
    i32.sub
    i32.store
    get_local $p0
    i32.load offset=20
    get_local $l0
    i32.gt_u
    if $I0
      get_local $p0
      get_local $l0
      i32.const 2
      i32.add
      i32.store offset=16
    end)
  (func $ff_hevc_sao_band_position_decode (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    call $get_cabac_bypass
    set_local $l0
    loop $L0
      get_local $p0
      i32.load offset=136
      i32.const 224
      i32.add
      call $get_cabac_bypass
      get_local $l0
      i32.const 1
      i32.shl
      i32.or
      set_local $l0
      get_local $l1
      i32.const 1
      i32.add
      tee_local $l1
      i32.const 4
      i32.ne
      br_if $L0
    end
    get_local $l0)
  (func $ff_hevc_sao_offset_abs_decode (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    block $B0
      i32.const 31
      i32.const -1
      get_local $p0
      i32.load offset=200
      i32.load offset=52
      tee_local $l1
      i32.const 5
      i32.sub
      i32.shl
      i32.const -1
      i32.xor
      get_local $l1
      i32.const 10
      i32.gt_s
      select
      tee_local $l1
      i32.const 1
      i32.lt_s
      br_if $B0
      loop $L1
        get_local $p0
        i32.load offset=136
        i32.const 224
        i32.add
        call $get_cabac_bypass
        i32.eqz
        br_if $B0
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $l1
        i32.ne
        br_if $L1
      end
      get_local $l1
      set_local $l0
    end
    get_local $l0)
  (func $ff_hevc_sao_offset_sign_decode (type $t0) (param $p0 i32) (result i32)
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    call $get_cabac_bypass)
  (func $ff_hevc_sao_eo_class_decode (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    call $get_cabac_bypass
    set_local $l0
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    call $get_cabac_bypass
    get_local $l0
    i32.const 1
    i32.shl
    i32.or)
  (func $ff_hevc_end_of_slice_flag_decode (type $t0) (param $p0 i32) (result i32)
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    call $get_cabac_terminate)
  (func $ff_hevc_cu_qp_delta_abs (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    i32.const 9
    set_local $l1
    block $B0
      loop $L1
        get_local $p0
        i32.load offset=136
        tee_local $l3
        i32.const 224
        i32.add
        get_local $l1
        get_local $l3
        i32.add
        call $get_cabac
        i32.eqz
        br_if $B0
        i32.const 10
        set_local $l1
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 5
        i32.ne
        br_if $L1
      end
      i32.const 0
      set_local $l1
      block $B2
        block $B3
          loop $L4
            get_local $p0
            i32.load offset=136
            i32.const 224
            i32.add
            call $get_cabac_bypass
            if $I5
              i32.const 1
              get_local $l1
              i32.shl
              get_local $l2
              i32.add
              set_local $l2
              i32.const 31
              set_local $l0
              get_local $l1
              i32.const 1
              i32.add
              tee_local $l1
              i32.const 31
              i32.ne
              br_if $L4
              br $B3
            end
          end
          get_local $l1
          i32.eqz
          br_if $B2
          get_local $l1
          set_local $l0
        end
        loop $L6
          get_local $p0
          i32.load offset=136
          i32.const 224
          i32.add
          call $get_cabac_bypass
          get_local $l0
          i32.const 1
          i32.sub
          tee_local $l0
          i32.shl
          get_local $l2
          i32.add
          set_local $l2
          get_local $l0
          br_if $L6
        end
      end
      i32.const 5
      set_local $l0
    end
    get_local $l0
    get_local $l2
    i32.add)
  (func $ff_hevc_cu_chroma_qp_offset_idx (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_local $p0
    i32.load offset=204
    i32.load8_u offset=1633
    tee_local $l0
    i32.const 5
    get_local $l0
    i32.const 5
    i32.gt_u
    select
    set_local $l0
    block $B0
      loop $L1
        get_local $p0
        i32.load offset=136
        tee_local $l2
        i32.const 224
        i32.add
        get_local $l2
        i32.const 177
        i32.add
        call $get_cabac
        i32.eqz
        br_if $B0
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        get_local $l0
        i32.ne
        br_if $L1
      end
      get_local $l0
      set_local $l1
    end
    get_local $l1)
  (func $ff_hevc_split_coding_unit_flag_decode (type $t9) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32)
    get_local $p3
    get_local $p0
    i32.load offset=200
    tee_local $l1
    i32.load offset=13064
    tee_local $l0
    i32.shr_s
    set_local $l2
    get_local $p2
    get_local $l0
    i32.shr_s
    set_local $l0
    i32.const -1
    get_local $l1
    i32.load offset=13080
    i32.shl
    i32.const -1
    i32.xor
    tee_local $l3
    get_local $p3
    i32.and
    set_local $l4
    get_local $p0
    i32.load offset=136
    set_local $p3
    block $B0 (result i32)
      get_local $p2
      get_local $l3
      i32.and
      i32.eqz
      if $I1
        i32.const 0
        get_local $p3
        i32.load8_u offset=308
        i32.eqz
        br_if $B0
        drop
      end
      get_local $p0
      i32.load offset=4336
      get_local $l0
      get_local $l1
      i32.load offset=13140
      get_local $l2
      i32.mul
      i32.add
      i32.add
      i32.const 1
      i32.sub
      i32.load8_u
    end
    set_local $p2
    get_local $p3
    i32.const 224
    i32.add
    get_local $p3
    block $B2 (result i32)
      get_local $l4
      i32.eqz
      if $I3
        i32.const 0
        get_local $p3
        i32.load8_u offset=309
        i32.eqz
        br_if $B2
        drop
      end
      get_local $p0
      i32.load offset=4336
      get_local $l1
      i32.load offset=13140
      get_local $l2
      i32.const 1
      i32.sub
      i32.mul
      get_local $l0
      i32.add
      i32.add
      i32.load8_u
    end
    get_local $p1
    i32.gt_s
    i32.const 3
    i32.const 2
    get_local $p1
    get_local $p2
    i32.lt_s
    select
    i32.add
    i32.add
    call $get_cabac)
  (func $ff_hevc_mpm_idx_decode (type $t0) (param $p0 i32) (result i32)
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    call $get_cabac_bypass
    if $I0
      i32.const 2
      i32.const 1
      get_local $p0
      i32.load offset=136
      i32.const 224
      i32.add
      call $get_cabac_bypass
      select
      return
    end
    i32.const 0)
  (func $ff_hevc_intra_chroma_pred_mode_decode (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    i32.load offset=136
    tee_local $l0
    i32.const 224
    i32.add
    get_local $l0
    i32.const 18
    i32.add
    call $get_cabac
    i32.eqz
    if $I0
      i32.const 4
      return
    end
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    call $get_cabac_bypass
    set_local $l0
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    call $get_cabac_bypass
    get_local $l0
    i32.const 1
    i32.shl
    i32.or)
  (func $ff_hevc_cbf_cb_cr_decode (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    get_local $p0
    i32.load offset=136
    tee_local $p0
    i32.const 224
    i32.add
    get_local $p0
    get_local $p1
    i32.add
    i32.const 42
    i32.add
    call $get_cabac)
  (func $ff_hevc_log2_res_scale_abs (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p1
    i32.const 2
    i32.shl
    i32.const 166
    i32.add
    set_local $l0
    i32.const 0
    set_local $p1
    loop $L0
      get_local $p0
      i32.load offset=136
      tee_local $l1
      i32.const 224
      i32.add
      get_local $l1
      get_local $p1
      get_local $l0
      i32.add
      i32.add
      call $get_cabac
      i32.eqz
      if $I1
        get_local $p1
        return
      end
      get_local $p1
      i32.const 1
      i32.add
      tee_local $p1
      i32.const 4
      i32.ne
      br_if $L0
    end
    i32.const 4)
  (func $ff_hevc_hls_residual_coding (type $t6) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32) (local $l13 i32) (local $l14 i32) (local $l15 i32) (local $l16 i32) (local $l17 i32) (local $l18 i32) (local $l19 i32) (local $l20 i32) (local $l21 i32) (local $l22 i32) (local $l23 i32) (local $l24 i32) (local $l25 i32) (local $l26 i32) (local $l27 i32) (local $l28 i32) (local $l29 i32) (local $l30 i32) (local $l31 i32) (local $l32 i32) (local $l33 i32) (local $l34 i32) (local $l35 i32) (local $l36 i32) (local $l37 i32) (local $l38 i32) (local $l39 i32) (local $l40 i32) (local $l41 i32) (local $l42 i32) (local $l43 i32) (local $l44 i32) (local $l45 i32) (local $l46 i32) (local $l47 i32) (local $l48 i32) (local $l49 i32) (local $l50 i32) (local $l51 i32) (local $l52 i32) (local $l53 i32) (local $l54 i32) (local $l55 i32) (local $l56 i32) (local $l57 i32) (local $l58 i32) (local $l59 i32) (local $l60 i32) (local $l61 i32) (local $l62 i64) (local $l63 i64) (local $l64 i64) (local $l65 i64)
    get_global $g0
    i32.const 96
    i32.sub
    tee_local $l11
    set_global $g0
    get_local $p5
    i32.const 2
    i32.shl
    tee_local $l0
    get_local $p0
    i32.load offset=160
    i32.add
    tee_local $l3
    i32.load
    set_local $l42
    get_local $l0
    get_local $p0
    i32.load offset=200
    tee_local $l5
    i32.add
    tee_local $l0
    i32.const 13180
    i32.add
    i32.load
    set_local $l43
    get_local $l0
    i32.const 13168
    i32.add
    i32.load
    set_local $l44
    get_local $l3
    i32.load offset=32
    set_local $l31
    get_local $l5
    i32.load offset=56
    set_local $l45
    get_local $p0
    i32.load offset=136
    set_local $l6
    get_local $l11
    i32.const 32
    i32.add
    i32.const 0
    i32.const 64
    call $memset
    drop
    get_local $l6
    i32.const 292
    i32.const 288
    get_local $p5
    select
    i32.add
    i32.load
    set_local $l29
    get_local $l6
    i32.const 11680
    i32.add
    get_local $l6
    i32.const 320
    i32.add
    tee_local $l46
    get_local $p5
    select
    i32.const 0
    i32.const 1
    get_local $p3
    i32.shl
    tee_local $l23
    get_local $p3
    i32.shl
    tee_local $l32
    i32.const 1
    i32.shl
    call $memset
    set_local $l14
    i32.const 0
    set_local $l5
    block $B0
      get_local $l6
      i32.const 31256
      i32.add
      i32.load8_u
      br_if $B0
      get_local $l6
      i32.load8_s offset=272
      set_local $l3
      block $B1
        get_local $p0
        i32.load offset=204
        tee_local $l0
        i32.load8_u offset=21
        i32.eqz
        br_if $B1
        get_local $l0
        i32.load8_u offset=1629
        get_local $p3
        i32.lt_s
        br_if $B1
        get_local $p0
        i32.load offset=136
        tee_local $l0
        i32.const 224
        i32.add
        get_local $l0
        i32.const 47
        i32.const 46
        get_local $p5
        select
        i32.add
        call $get_cabac
        set_local $l18
      end
      block $B2 (result i32)
        get_local $p5
        i32.eqz
        if $I3
          get_local $p0
          i32.load offset=200
          tee_local $l0
          i32.load offset=13192
          get_local $l3
          i32.add
          br $B2
        end
        get_local $p0
        i32.load offset=204
        set_local $l0
        block $B4 (result i32)
          get_local $p5
          i32.const 1
          i32.eq
          if $I5
            get_local $l6
            i32.const 302
            i32.add
            set_local $l8
            get_local $p0
            i32.const 2072
            i32.add
            i32.load
            get_local $l0
            i32.load offset=28
            i32.add
            br $B4
          end
          get_local $l6
          i32.const 303
          i32.add
          set_local $l8
          get_local $p0
          i32.const 2076
          i32.add
          i32.load
          get_local $l0
          i32.load offset=32
          i32.add
        end
        set_local $l5
        i32.const 0
        get_local $p0
        i32.load offset=200
        tee_local $l0
        i32.load offset=13192
        tee_local $l7
        i32.sub
        tee_local $l1
        get_local $l5
        get_local $l8
        i32.load8_s
        i32.add
        get_local $l3
        i32.add
        tee_local $l3
        i32.const 57
        get_local $l3
        i32.const 57
        i32.lt_s
        select
        get_local $l1
        get_local $l3
        i32.gt_s
        select
        set_local $l1
        block $B6
          get_local $l0
          i32.load offset=4
          i32.const 1
          i32.eq
          if $I7
            get_local $l1
            i32.const 30
            i32.lt_s
            br_if $B6
            get_local $l1
            i32.const 44
            i32.ge_s
            if $I8
              get_local $l1
              i32.const 6
              i32.sub
              set_local $l1
              br $B6
            end
            get_local $l1
            i32.const 2
            i32.shl
            i32.const 1080
            i32.add
            i32.load
            set_local $l1
            br $B6
          end
          get_local $l1
          i32.const 51
          get_local $l1
          i32.const 51
          i32.lt_s
          select
          set_local $l1
        end
        get_local $l1
        get_local $l7
        i32.add
      end
      tee_local $l3
      i32.const 1264
      i32.add
      i32.load8_u
      i32.const 1184
      i32.add
      i32.load8_u
      get_local $l3
      i32.const 1344
      i32.add
      i32.load8_u
      i32.shl
      set_local $l8
      get_local $l0
      i32.load offset=52
      get_local $p3
      i32.add
      tee_local $l3
      i32.const 5
      i32.sub
      set_local $l13
      i32.const 1
      get_local $l3
      i32.const 6
      i32.sub
      i32.shl
      set_local $l5
      i32.const 16
      set_local $l33
      get_local $l0
      i32.load8_u offset=634
      i32.eqz
      get_local $p3
      i32.const 3
      i32.ge_s
      i32.const 0
      get_local $l18
      select
      i32.or
      br_if $B0
      get_local $p0
      i32.load offset=204
      tee_local $l3
      i32.const 69
      i32.add
      get_local $l0
      i32.const 635
      i32.add
      get_local $l3
      i32.load8_u offset=68
      select
      tee_local $l0
      get_local $p3
      i32.const 384
      i32.mul
      i32.add
      i32.const 0
      i32.const 3
      get_local $l6
      i32.const 31244
      i32.add
      i32.load
      i32.const 1
      i32.eq
      select
      get_local $p5
      i32.add
      tee_local $l3
      i32.const 6
      i32.shl
      i32.add
      i32.const 768
      i32.sub
      set_local $l47
      get_local $p3
      i32.const 4
      i32.lt_s
      br_if $B0
      get_local $p3
      i32.const 6
      i32.mul
      get_local $l0
      i32.add
      get_local $l3
      i32.add
      i32.const 1512
      i32.add
      i32.load8_u
      set_local $l33
    end
    get_local $p3
    i32.const 1
    i32.shl
    set_local $l0
    block $B9 (result i32)
      get_local $p5
      i32.eqz
      if $I10
        get_local $p3
        i32.const 3
        i32.mul
        get_local $p3
        i32.const 1
        i32.sub
        i32.const 2
        i32.shr_s
        i32.add
        i32.const 6
        i32.sub
        set_local $l2
        get_local $p3
        i32.const 1
        i32.add
        i32.const 2
        i32.shr_s
        br $B9
      end
      i32.const 15
      set_local $l2
      get_local $p3
      i32.const 2
      i32.sub
    end
    set_local $l1
    i32.const 0
    set_local $l7
    i32.const 0
    set_local $l3
    block $B11
      get_local $l0
      i32.const 2
      i32.lt_s
      br_if $B11
      get_local $l0
      i32.const 1
      i32.sub
      set_local $l0
      get_local $l2
      i32.const 52
      i32.add
      set_local $l3
      block $B12
        loop $L13
          get_local $p0
          i32.load offset=136
          tee_local $l12
          i32.const 224
          i32.add
          get_local $l12
          get_local $l3
          get_local $l7
          get_local $l1
          i32.shr_u
          i32.add
          i32.add
          call $get_cabac
          i32.eqz
          br_if $B12
          get_local $l7
          i32.const 1
          i32.add
          tee_local $l7
          get_local $l0
          i32.ne
          br_if $L13
        end
        get_local $l0
        set_local $l7
      end
      get_local $l2
      i32.const 70
      i32.add
      set_local $l2
      i32.const 0
      set_local $l3
      block $B14
        loop $L15
          get_local $p0
          i32.load offset=136
          tee_local $l12
          i32.const 224
          i32.add
          get_local $l12
          get_local $l2
          get_local $l3
          get_local $l1
          i32.shr_u
          i32.add
          i32.add
          call $get_cabac
          i32.eqz
          br_if $B14
          get_local $l3
          i32.const 1
          i32.add
          tee_local $l3
          get_local $l0
          i32.ne
          br_if $L15
        end
        get_local $l0
        set_local $l3
      end
      get_local $l7
      i32.const 4
      i32.ge_s
      if $I16
        get_local $l7
        i32.const 1
        i32.shr_u
        set_local $l2
        get_local $p0
        i32.load offset=136
        i32.const 224
        i32.add
        call $get_cabac_bypass
        set_local $l0
        get_local $l7
        i32.const 6
        i32.ge_s
        if $I17
          get_local $l2
          i32.const 3
          get_local $l2
          i32.const 3
          i32.gt_u
          select
          i32.const 1
          i32.sub
          set_local $l12
          i32.const 1
          set_local $l1
          loop $L18
            get_local $p0
            i32.load offset=136
            i32.const 224
            i32.add
            call $get_cabac_bypass
            get_local $l0
            i32.const 1
            i32.shl
            i32.or
            set_local $l0
            get_local $l1
            i32.const 1
            i32.add
            tee_local $l1
            get_local $l12
            i32.ne
            br_if $L18
          end
        end
        get_local $l0
        get_local $l7
        i32.const 1
        i32.and
        i32.const 2
        i32.or
        get_local $l2
        i32.const 1
        i32.sub
        i32.shl
        i32.add
        set_local $l7
      end
      get_local $l3
      i32.const 4
      i32.lt_s
      br_if $B11
      get_local $l3
      i32.const 1
      i32.shr_u
      set_local $l2
      get_local $p0
      i32.load offset=136
      i32.const 224
      i32.add
      call $get_cabac_bypass
      set_local $l0
      get_local $l3
      i32.const 6
      i32.ge_s
      if $I19
        get_local $l2
        i32.const 3
        get_local $l2
        i32.const 3
        i32.gt_u
        select
        i32.const 1
        i32.sub
        set_local $l12
        i32.const 1
        set_local $l1
        loop $L20
          get_local $p0
          i32.load offset=136
          i32.const 224
          i32.add
          call $get_cabac_bypass
          get_local $l0
          i32.const 1
          i32.shl
          i32.or
          set_local $l0
          get_local $l1
          i32.const 1
          i32.add
          tee_local $l1
          get_local $l12
          i32.ne
          br_if $L20
        end
      end
      get_local $l0
      get_local $l3
      i32.const 1
      i32.and
      i32.const 2
      i32.or
      get_local $l2
      i32.const 1
      i32.sub
      i32.shl
      i32.add
      set_local $l3
    end
    block $B21
      block $B22
        get_local $p4
        i32.const 2
        i32.eq
        if $I23
          get_local $l7
          i32.const 2
          i32.shr_s
          set_local $l24
          get_local $l3
          i32.const 2
          i32.shr_s
          set_local $l25
          get_local $l7
          set_local $l0
          get_local $l3
          set_local $l7
          get_local $l0
          set_local $l3
          br $B22
        end
        get_local $l3
        i32.const 2
        i32.shr_s
        set_local $l24
        get_local $l7
        i32.const 2
        i32.shr_s
        set_local $l25
        get_local $l3
        set_local $l0
        block $B24
          block $B25
            get_local $p4
            br_table $B25 $B24 $B22
          end
          get_local $l7
          i32.const 3
          i32.and
          get_local $l3
          i32.const 3
          i32.and
          i32.const 2
          i32.shl
          i32.or
          i32.const 1424
          i32.add
          i32.load8_u
          set_local $l16
          i32.const 1440
          set_local $l20
          i32.const 1024
          set_local $l27
          i32.const 1040
          set_local $l28
          i32.const 1440
          set_local $l21
          block $B26
            block $B27
              block $B28
                get_local $p3
                i32.const 2
                i32.sub
                br_table $B21 $B28 $B27 $B26
              end
              get_local $l24
              i32.const 1
              i32.shl
              get_local $l25
              i32.add
              i32.const 1441
              i32.add
              i32.load8_u
              i32.const 4
              i32.shl
              get_local $l16
              i32.add
              set_local $l16
              i32.const 1524
              set_local $l21
              i32.const 1520
              set_local $l20
              br $B21
            end
            get_local $l24
            i32.const 2
            i32.shl
            get_local $l25
            i32.add
            i32.const 1424
            i32.add
            i32.load8_u
            i32.const 4
            i32.shl
            get_local $l16
            i32.add
            set_local $l16
            i32.const 1040
            set_local $l20
            i32.const 1024
            set_local $l21
            br $B21
          end
          get_local $l24
          i32.const 3
          i32.shl
          get_local $l25
          i32.add
          i32.const 1456
          i32.add
          i32.load8_u
          i32.const 4
          i32.shl
          get_local $l16
          i32.add
          set_local $l16
          i32.const 1056
          set_local $l21
          i32.const 1120
          set_local $l20
          br $B21
        end
        get_local $l3
        i32.const 3
        i32.shl
        get_local $l7
        i32.add
        i32.const 1568
        i32.add
        i32.load8_u
        set_local $l16
        i32.const 1520
        set_local $l21
        i32.const 1524
        set_local $l20
        i32.const 1536
        set_local $l27
        i32.const 1552
        set_local $l28
        br $B21
      end
      get_local $l7
      i32.const 3
      i32.shl
      get_local $l0
      i32.add
      i32.const 1568
      i32.add
      i32.load8_u
      set_local $l16
      i32.const 1524
      set_local $l21
      i32.const 1520
      set_local $l20
      i32.const 1552
      set_local $l27
      i32.const 1536
      set_local $l28
    end
    i32.const 1
    i32.const 3
    get_local $p5
    select
    set_local $l48
    i32.const 43
    i32.const 42
    get_local $p5
    select
    set_local $l34
    i32.const 15
    i32.const 9
    get_local $p4
    select
    set_local $l49
    i32.const 41
    i32.const 40
    get_local $p5
    select
    set_local $l35
    get_local $l18
    i32.const 0
    i32.ne
    get_local $p3
    i32.const 2
    i32.gt_s
    i32.and
    set_local $l50
    i32.const 27
    i32.const 0
    get_local $p5
    select
    tee_local $l12
    i32.const 3
    i32.add
    set_local $l51
    get_local $l12
    i32.const 9
    i32.const 12
    get_local $p3
    i32.const 3
    i32.eq
    select
    i32.add
    set_local $l52
    i32.const 90
    i32.const 88
    get_local $p5
    i32.const 0
    i32.gt_s
    tee_local $l36
    select
    set_local $l53
    get_local $p5
    i32.eqz
    i32.const 1
    i32.shl
    set_local $l54
    get_local $l23
    i32.const 1
    i32.sub
    i32.const 2
    i32.shr_s
    set_local $l37
    i32.const -1
    get_local $p3
    i32.const 2
    i32.sub
    tee_local $l30
    i32.shl
    i32.const -1
    i32.xor
    set_local $l38
    get_local $l13
    i64.extend_u/i32
    set_local $l63
    get_local $l5
    i64.extend_s/i32
    set_local $l64
    get_local $l8
    i64.extend_s/i32
    set_local $l65
    get_local $l29
    i32.const -17
    i32.and
    tee_local $l39
    i32.const 10
    i32.ne
    set_local $l55
    i32.const 1
    set_local $l13
    get_local $l16
    i32.const 4
    i32.shr_u
    tee_local $l40
    set_local $l15
    i32.const 16
    set_local $l23
    loop $L29
      get_local $l15
      get_local $l20
      i32.add
      i32.load8_u
      set_local $l9
      get_local $l15
      get_local $l21
      i32.add
      i32.load8_u
      set_local $l10
      block $B30 (result i32)
        get_local $l15
        i32.eqz
        get_local $l15
        get_local $l40
        i32.ge_s
        i32.or
        i32.eqz
        if $I31
          i32.const 0
          set_local $l0
          get_local $l10
          get_local $l38
          i32.lt_s
          if $I32
            get_local $l10
            i32.const 3
            i32.shl
            get_local $l11
            i32.add
            get_local $l9
            i32.add
            i32.load8_u offset=40
            set_local $l0
          end
          i32.const 1
          set_local $l8
          get_local $p0
          i32.load offset=136
          tee_local $p4
          i32.const 224
          i32.add
          get_local $p4
          block $B33 (result i32)
            get_local $l9
            get_local $l38
            i32.lt_s
            if $I34
              get_local $l0
              get_local $l9
              get_local $l10
              i32.const 3
              i32.shl
              get_local $l11
              i32.add
              i32.add
              i32.load8_u offset=33
              i32.add
              set_local $l0
            end
            get_local $l0
          end
          i32.const 1
          get_local $l0
          i32.const 1
          i32.lt_s
          select
          get_local $l53
          i32.add
          i32.add
          call $get_cabac
          br $B30
        end
        i32.const 0
        set_local $l8
        block $B35 (result i32)
          get_local $l10
          get_local $l25
          i32.eq
          if $I36
            i32.const 1
            get_local $l9
            get_local $l24
            i32.eq
            br_if $B35
            drop
          end
          get_local $l9
          get_local $l10
          i32.or
          i32.eqz
        end
      end
      set_local $l5
      get_local $l10
      i32.const 3
      i32.shl
      tee_local $l2
      get_local $l11
      i32.const 32
      i32.add
      i32.add
      get_local $l9
      i32.add
      tee_local $l4
      get_local $l5
      i32.store8
      i32.const 15
      set_local $l0
      i32.const 0
      set_local $p4
      i32.const 0
      set_local $l1
      get_local $l15
      get_local $l40
      i32.ne
      tee_local $l19
      i32.eqz
      if $I37
        get_local $l11
        get_local $l16
        get_local $l15
        i32.const 4
        i32.shl
        i32.sub
        tee_local $l0
        i32.store8 offset=16
        i32.const 1
        set_local $l1
        get_local $l0
        i32.const 1
        i32.sub
        set_local $l0
      end
      get_local $l10
      get_local $l37
      i32.lt_s
      if $I38
        get_local $l2
        get_local $l11
        i32.add
        get_local $l9
        i32.add
        i32.load8_u offset=40
        i32.const 0
        i32.ne
        set_local $p4
      end
      get_local $l9
      get_local $l37
      i32.lt_s
      if $I39
        get_local $l4
        i32.load8_u offset=1
        i32.const 0
        i32.ne
        i32.const 1
        i32.shl
        get_local $p4
        i32.or
        set_local $p4
      end
      block $B40
        get_local $l5
        i32.const 255
        i32.and
        i32.eqz
        get_local $l0
        i32.const 0
        i32.lt_s
        i32.or
        br_if $B40
        block $B41
          block $B42
            get_local $p0
            i32.load offset=200
            i32.load offset=13100
            if $I43
              i32.const 1696
              set_local $l4
              get_local $l35
              set_local $l5
              get_local $l18
              br_if $B41
              get_local $l35
              get_local $l12
              get_local $l6
              i32.load8_u offset=31256
              tee_local $l2
              select
              set_local $l5
              i32.const 1696
              i32.const 1632
              get_local $l2
              select
              set_local $l4
              get_local $l2
              br_if $B41
              get_local $p3
              i32.const 2
              i32.ne
              br_if $B42
              br $B41
            end
            i32.const 1632
            set_local $l4
            get_local $l12
            set_local $l5
            get_local $p3
            i32.const 2
            i32.eq
            br_if $B41
          end
          get_local $p4
          i32.const 4
          i32.shl
          i32.const 1648
          i32.add
          set_local $l4
          get_local $p5
          if $I44
            get_local $l52
            set_local $l5
            br $B41
          end
          get_local $l51
          get_local $l12
          get_local $l9
          get_local $l10
          i32.or
          select
          set_local $p4
          get_local $p3
          i32.const 3
          i32.eq
          if $I45
            get_local $p4
            get_local $l49
            i32.add
            set_local $l5
            br $B41
          end
          get_local $p4
          i32.const 21
          i32.add
          set_local $l5
        end
        get_local $l0
        i32.const 1
        i32.ge_s
        if $I46
          get_local $l5
          i32.const 92
          i32.add
          set_local $p4
          loop $L47
            get_local $p0
            i32.load offset=136
            tee_local $l2
            i32.const 224
            i32.add
            get_local $l2
            get_local $p4
            get_local $l4
            get_local $l0
            get_local $l27
            i32.add
            i32.load8_u
            get_local $l0
            get_local $l28
            i32.add
            i32.load8_u
            i32.const 2
            i32.shl
            i32.add
            i32.add
            i32.load8_u
            i32.add
            i32.add
            call $get_cabac
            if $I48
              get_local $l11
              i32.const 16
              i32.add
              get_local $l1
              i32.const 255
              i32.and
              i32.add
              get_local $l0
              i32.store8
              i32.const 0
              set_local $l8
              get_local $l1
              i32.const 1
              i32.add
              set_local $l1
            end
            get_local $l0
            i32.const 1
            i32.gt_s
            set_local $l2
            get_local $l0
            i32.const 1
            i32.sub
            set_local $l0
            get_local $l2
            br_if $L47
          end
        end
        get_local $l8
        i32.eqz
        if $I49
          get_local $p0
          i32.load offset=136
          tee_local $p4
          i32.const 224
          i32.add
          block $B50 (result i32)
            get_local $p0
            i32.load offset=200
            i32.load offset=13100
            if $I51
              get_local $l34
              get_local $l18
              br_if $B50
              drop
              get_local $l34
              get_local $l6
              i32.load8_u offset=31256
              br_if $B50
              drop
            end
            get_local $l5
            i32.const 2
            i32.add
            get_local $l12
            get_local $l15
            select
          end
          get_local $p4
          i32.add
          i32.const 92
          i32.add
          call $get_cabac
          i32.const 1
          i32.ne
          br_if $B40
        end
        get_local $l11
        i32.const 16
        i32.add
        get_local $l1
        i32.const 255
        i32.and
        i32.add
        i32.const 0
        i32.store8
        get_local $l1
        i32.const 1
        i32.add
        set_local $l1
      end
      get_local $l1
      i32.const 255
      i32.and
      tee_local $p4
      if $I52
        i32.const 0
        set_local $l0
        i32.const 0
        set_local $l4
        get_local $p0
        i32.load offset=200
        i32.load offset=13116
        if $I53
          block $B54 (result i32)
            get_local $l18
            i32.eqz
            if $I55
              get_local $l54
              get_local $l6
              i32.load8_u offset=31256
              i32.eqz
              br_if $B54
              drop
            end
            get_local $l48
          end
          tee_local $l56
          get_local $l6
          i32.add
          i32.load8_u offset=199
          i32.const 2
          i32.shr_u
          set_local $l4
        end
        i32.const 1
        set_local $l5
        get_local $p4
        i32.const 8
        get_local $p4
        i32.const 8
        i32.lt_u
        select
        tee_local $l1
        i32.const 1
        get_local $l1
        i32.const 1
        i32.gt_s
        select
        set_local $l8
        get_local $l19
        get_local $l13
        i32.eqz
        i32.and
        i32.const 0
        get_local $l15
        i32.const 0
        i32.ne
        i32.const 1
        i32.shl
        get_local $p5
        select
        i32.or
        tee_local $l2
        i32.const 2
        i32.shl
        set_local $l19
        i32.const -1
        set_local $l13
        get_local $l11
        i32.load8_u offset=16
        set_local $l1
        loop $L56
          get_local $l11
          i32.const 8
          i32.add
          get_local $l0
          i32.add
          get_local $p0
          i32.load offset=136
          tee_local $l22
          i32.const 224
          i32.add
          get_local $l22
          get_local $l5
          get_local $l19
          i32.add
          tee_local $l17
          i32.const 16
          i32.add
          get_local $l17
          get_local $l36
          select
          i32.add
          i32.const 136
          i32.add
          call $get_cabac
          tee_local $l22
          i32.store8
          block $B57 (result i32)
            get_local $l22
            i32.const 255
            i32.and
            if $I58
              get_local $l0
              get_local $l13
              get_local $l13
              i32.const -1
              i32.eq
              select
              set_local $l13
              i32.const 0
              br $B57
            end
            get_local $l5
            get_local $l5
            i32.const 1
            i32.sub
            i32.const 2
            i32.lt_u
            i32.add
          end
          set_local $l5
          get_local $l0
          i32.const 1
          i32.add
          tee_local $l0
          get_local $l8
          i32.ne
          br_if $L56
        end
        get_local $p4
        i32.const 1
        i32.sub
        tee_local $l17
        get_local $l11
        i32.const 16
        i32.add
        i32.add
        i32.load8_u
        set_local $l22
        i32.const 0
        set_local $l19
        block $B59
          get_local $l6
          i32.load8_u offset=31256
          br_if $B59
          get_local $l18
          i32.eqz
          get_local $l6
          i32.load offset=31244
          i32.const 1
          i32.ne
          get_local $l55
          i32.or
          i32.or
          i32.eqz
          if $I60
            get_local $p0
            i32.load offset=200
            i32.load offset=13104
            br_if $B59
          end
          get_local $l1
          get_local $l22
          i32.sub
          i32.const 3
          i32.gt_s
          set_local $l19
        end
        get_local $l13
        i32.const -1
        i32.ne
        if $I61
          get_local $l11
          i32.const 8
          i32.add
          get_local $l13
          i32.add
          tee_local $l0
          get_local $p0
          i32.load offset=136
          tee_local $l8
          i32.const 224
          i32.add
          get_local $l8
          get_local $l2
          i32.const 4
          i32.or
          get_local $l2
          get_local $l36
          select
          i32.const 160
          i32.or
          i32.add
          call $get_cabac
          get_local $l0
          i32.load8_u
          i32.add
          i32.store8
        end
        i32.const 0
        set_local $l0
        block $B62
          block $B63
            block $B64
              get_local $l19
              i32.eqz
              if $I65
                i32.const 0
                set_local $l2
                br $B64
              end
              i32.const 0
              set_local $l2
              get_local $p0
              i32.load offset=204
              i32.load8_u offset=4
              br_if $B63
            end
            loop $L66
              get_local $p0
              i32.load offset=136
              i32.const 224
              i32.add
              call $get_cabac_bypass
              get_local $l0
              i32.const 1
              i32.shl
              i32.or
              set_local $l0
              get_local $l2
              i32.const 1
              i32.add
              tee_local $l2
              get_local $p4
              i32.ne
              br_if $L66
            end
            i32.const 16
            set_local $l8
            br $B62
          end
          i32.const 17
          set_local $l8
          get_local $l17
          i32.const 255
          i32.and
          tee_local $l17
          i32.eqz
          br_if $B62
          loop $L67
            get_local $p0
            i32.load offset=136
            i32.const 224
            i32.add
            call $get_cabac_bypass
            get_local $l0
            i32.const 1
            i32.shl
            i32.or
            set_local $l0
            get_local $l2
            i32.const 1
            i32.add
            tee_local $l2
            get_local $l17
            i32.ne
            br_if $L67
          end
        end
        get_local $p4
        i32.const 1
        get_local $p4
        i32.const 1
        i32.gt_u
        select
        set_local $l57
        get_local $l9
        i32.const 2
        i32.shl
        set_local $l58
        get_local $l10
        i32.const 2
        i32.shl
        set_local $l59
        get_local $l6
        get_local $l56
        i32.add
        set_local $l9
        get_local $l0
        get_local $l8
        get_local $p4
        i32.sub
        i32.shl
        set_local $l8
        i32.const 0
        set_local $l17
        i32.const 0
        set_local $l10
        i32.const 0
        set_local $p4
        loop $L68
          get_local $l28
          get_local $l1
          i32.const 255
          i32.and
          tee_local $l41
          i32.add
          i32.load8_u
          set_local $l60
          get_local $l27
          get_local $l41
          i32.add
          i32.load8_u
          set_local $l61
          i32.const 0
          set_local $l1
          block $B69
            block $B70
              block $B71
                get_local $p4
                i32.const 7
                i32.le_u
                if $I72
                  get_local $l11
                  i32.const 8
                  i32.add
                  get_local $p4
                  i32.add
                  i64.load8_u
                  i64.const 1
                  i64.add
                  tee_local $l62
                  i64.const 3
                  i64.const 2
                  get_local $p4
                  get_local $l13
                  i32.eq
                  select
                  i64.ne
                  br_if $B69
                  loop $L73
                    get_local $p0
                    i32.load offset=136
                    i32.const 224
                    i32.add
                    call $get_cabac_bypass
                    if $I74
                      i32.const 31
                      set_local $l0
                      get_local $l1
                      i32.const 1
                      i32.add
                      tee_local $l1
                      i32.const 31
                      i32.ne
                      br_if $L73
                      br $B71
                    end
                  end
                  get_local $l1
                  i32.const 2
                  i32.gt_u
                  if $I75
                    get_local $l1
                    set_local $l0
                    br $B71
                  end
                  i32.const 0
                  set_local $l0
                  i32.const 0
                  set_local $l2
                  get_local $l4
                  i32.const 1
                  i32.lt_s
                  br_if $B70
                  loop $L76
                    get_local $p0
                    i32.load offset=136
                    i32.const 224
                    i32.add
                    call $get_cabac_bypass
                    get_local $l2
                    i32.const 1
                    i32.shl
                    i32.or
                    set_local $l2
                    get_local $l0
                    i32.const 1
                    i32.add
                    tee_local $l0
                    get_local $l4
                    i32.ne
                    br_if $L76
                  end
                  get_local $l2
                  set_local $l0
                  br $B70
                end
                block $B77
                  block $B78
                    loop $L79
                      get_local $p0
                      i32.load offset=136
                      i32.const 224
                      i32.add
                      call $get_cabac_bypass
                      if $I80
                        i32.const 31
                        set_local $l0
                        get_local $l1
                        i32.const 1
                        i32.add
                        tee_local $l1
                        i32.const 31
                        i32.ne
                        br_if $L79
                        br $B78
                      end
                    end
                    get_local $l1
                    i32.const 2
                    i32.gt_u
                    if $I81
                      get_local $l1
                      set_local $l0
                      br $B78
                    end
                    i32.const 0
                    set_local $l0
                    i32.const 0
                    set_local $l2
                    get_local $l4
                    i32.const 1
                    i32.lt_s
                    br_if $B77
                    loop $L82
                      get_local $p0
                      i32.load offset=136
                      i32.const 224
                      i32.add
                      call $get_cabac_bypass
                      get_local $l2
                      i32.const 1
                      i32.shl
                      i32.or
                      set_local $l2
                      get_local $l0
                      i32.const 1
                      i32.add
                      tee_local $l0
                      get_local $l4
                      i32.ne
                      br_if $L82
                    end
                    get_local $l2
                    set_local $l0
                    br $B77
                  end
                  block $B83
                    get_local $l0
                    i32.const 3
                    i32.sub
                    tee_local $l2
                    get_local $l4
                    i32.add
                    i32.const 1
                    i32.lt_s
                    if $I84
                      i32.const 0
                      set_local $l0
                      br $B83
                    end
                    get_local $l0
                    get_local $l4
                    i32.add
                    i32.const 3
                    i32.sub
                    set_local $l26
                    i32.const 0
                    set_local $l1
                    i32.const 0
                    set_local $l0
                    loop $L85
                      get_local $p0
                      i32.load offset=136
                      i32.const 224
                      i32.add
                      call $get_cabac_bypass
                      get_local $l0
                      i32.const 1
                      i32.shl
                      i32.or
                      set_local $l0
                      get_local $l1
                      i32.const 1
                      i32.add
                      tee_local $l1
                      get_local $l26
                      i32.ne
                      br_if $L85
                    end
                  end
                  i32.const 1
                  get_local $l2
                  i32.shl
                  i32.const 2
                  i32.add
                  set_local $l1
                end
                get_local $l0
                get_local $l1
                get_local $l4
                i32.shl
                i32.add
                tee_local $l0
                i32.const 1
                i32.add
                set_local $l2
                get_local $p0
                i32.load offset=200
                i32.load offset=13116
                set_local $l1
                i32.const 3
                get_local $l4
                i32.shl
                get_local $l0
                i32.le_s
                if $I86
                  get_local $l4
                  i32.const 1
                  i32.add
                  tee_local $l26
                  get_local $l26
                  i32.const 4
                  get_local $l4
                  i32.const 4
                  i32.lt_s
                  select
                  get_local $l1
                  select
                  set_local $l4
                end
                get_local $l2
                i64.extend_s/i32
                set_local $l62
                get_local $l10
                get_local $l1
                i32.eqz
                i32.or
                br_if $B69
                i32.const 3
                get_local $l9
                i32.load8_u offset=199
                tee_local $l1
                i32.const 2
                i32.shr_u
                tee_local $l2
                i32.shl
                get_local $l0
                i32.le_s
                if $I87
                  i32.const 1
                  set_local $l10
                  get_local $l9
                  get_local $l1
                  i32.const 1
                  i32.add
                  i32.store8 offset=199
                  br $B69
                end
                i32.const 1
                set_local $l10
                get_local $l1
                i32.eqz
                get_local $l0
                i32.const 1
                i32.shl
                i32.const 1
                get_local $l2
                i32.shl
                i32.ge_s
                i32.or
                br_if $B69
                get_local $l9
                get_local $l1
                i32.const 1
                i32.sub
                i32.store8 offset=199
                br $B69
              end
              block $B88
                get_local $l0
                i32.const 3
                i32.sub
                tee_local $l2
                get_local $l4
                i32.add
                i32.const 1
                i32.lt_s
                if $I89
                  i32.const 0
                  set_local $l0
                  br $B88
                end
                get_local $l0
                get_local $l4
                i32.add
                i32.const 3
                i32.sub
                set_local $l26
                i32.const 0
                set_local $l1
                i32.const 0
                set_local $l0
                loop $L90
                  get_local $p0
                  i32.load offset=136
                  i32.const 224
                  i32.add
                  call $get_cabac_bypass
                  get_local $l0
                  i32.const 1
                  i32.shl
                  i32.or
                  set_local $l0
                  get_local $l1
                  i32.const 1
                  i32.add
                  tee_local $l1
                  get_local $l26
                  i32.ne
                  br_if $L90
                end
              end
              i32.const 1
              get_local $l2
              i32.shl
              i32.const 2
              i32.add
              set_local $l1
            end
            get_local $p0
            i32.load offset=200
            i32.load offset=13116
            set_local $l2
            get_local $l62
            get_local $l0
            get_local $l1
            get_local $l4
            i32.shl
            i32.add
            tee_local $l1
            i64.extend_s/i32
            i64.add
            tee_local $l62
            i32.const 3
            get_local $l4
            i32.shl
            i64.extend_s/i32
            i64.gt_s
            if $I91
              get_local $l4
              i32.const 1
              i32.add
              tee_local $l0
              get_local $l0
              i32.const 4
              get_local $l4
              i32.const 4
              i32.lt_s
              select
              get_local $l2
              select
              set_local $l4
            end
            get_local $l10
            get_local $l2
            i32.eqz
            i32.or
            br_if $B69
            i32.const 3
            get_local $l9
            i32.load8_u offset=199
            tee_local $l0
            i32.const 2
            i32.shr_u
            tee_local $l2
            i32.shl
            get_local $l1
            i32.le_s
            if $I92
              i32.const 1
              set_local $l10
              get_local $l9
              get_local $l0
              i32.const 1
              i32.add
              i32.store8 offset=199
              br $B69
            end
            i32.const 1
            set_local $l10
            get_local $l0
            i32.eqz
            get_local $l1
            i32.const 1
            i32.shl
            i32.const 1
            get_local $l2
            i32.shl
            i32.ge_s
            i32.or
            br_if $B69
            get_local $l9
            get_local $l0
            i32.const 1
            i32.sub
            i32.store8 offset=199
          end
          block $B93
            get_local $l19
            i32.eqz
            br_if $B93
            get_local $p0
            i32.load offset=204
            i32.load8_u offset=4
            i32.eqz
            br_if $B93
            get_local $l62
            i64.const 0
            get_local $l62
            i64.sub
            get_local $l62
            get_local $l17
            get_local $l62
            i32.wrap/i64
            i32.add
            tee_local $l17
            i32.const 1
            i32.and
            select
            get_local $l22
            get_local $l41
            i32.ne
            select
            set_local $l62
          end
          get_local $l14
          get_local $l59
          get_local $l61
          i32.add
          tee_local $l0
          get_local $l58
          get_local $l60
          i32.add
          tee_local $l1
          get_local $p3
          i32.shl
          i32.add
          i32.const 1
          i32.shl
          i32.add
          block $B94 (result i64)
            i64.const 0
            get_local $l62
            i64.sub
            get_local $l62
            get_local $l8
            i32.const 32768
            i32.and
            select
            tee_local $l62
            get_local $l6
            i32.load8_u offset=31256
            br_if $B94
            drop
            block $B95
              get_local $p0
              i32.load offset=200
              i32.load8_u offset=634
              i32.eqz
              get_local $l50
              i32.or
              br_if $B95
              get_local $p3
              i32.const 4
              i32.ge_s
              if $I96
                get_local $l33
                set_local $l23
                get_local $l0
                get_local $l1
                i32.or
                i32.eqz
                br_if $B95
              end
              block $B97 (result i32)
                block $B98
                  block $B99
                    block $B100
                      block $B101
                        get_local $p3
                        i32.const 3
                        i32.sub
                        br_table $B101 $B100 $B99 $B98
                      end
                      get_local $l1
                      i32.const 3
                      i32.shl
                      get_local $l0
                      i32.add
                      br $B97
                    end
                    get_local $l1
                    i32.const 2
                    i32.shl
                    i32.const 8184
                    i32.and
                    get_local $l0
                    i32.const 1
                    i32.shr_u
                    i32.add
                    br $B97
                  end
                  get_local $l1
                  i32.const 1
                  i32.shl
                  i32.const 4088
                  i32.and
                  get_local $l0
                  i32.const 2
                  i32.shr_u
                  i32.add
                  br $B97
                end
                get_local $l1
                i32.const 2
                i32.shl
                get_local $l0
                i32.add
              end
              get_local $l47
              i32.add
              i32.load8_u
              set_local $l23
            end
            get_local $l23
            i64.extend_s/i32
            get_local $l62
            get_local $l65
            i64.mul
            i64.mul
            get_local $l64
            i64.add
            get_local $l63
            i64.shr_s
            tee_local $l62
            i64.const -1
            i64.le_s
            if $I102
              get_local $l62
              i64.const -32768
              get_local $l62
              i64.const 1152921504606814208
              i64.and
              i64.const 1152921504606814208
              i64.eq
              select
              br $B94
            end
            get_local $l62
            i64.const 32767
            get_local $l62
            i64.const 32767
            i64.lt_u
            select
          end
          i64.store16
          get_local $l57
          get_local $p4
          i32.const 1
          i32.add
          tee_local $p4
          i32.ne
          if $I103
            get_local $l8
            i32.const 1
            i32.shl
            i32.const 131070
            i32.and
            set_local $l8
            get_local $l11
            i32.const 16
            i32.add
            get_local $p4
            i32.add
            i32.load8_u
            set_local $l1
            br $L68
          end
        end
        get_local $l5
        set_local $l13
      end
      get_local $l15
      i32.const 0
      i32.gt_s
      set_local $p4
      get_local $l15
      i32.const 1
      i32.sub
      set_local $l15
      get_local $p4
      br_if $L29
    end
    block $B104
      get_local $l6
      i32.load8_u offset=31256
      if $I105
        get_local $l39
        i32.const 10
        i32.ne
        br_if $B104
        get_local $p0
        i32.load offset=200
        i32.load offset=13104
        i32.eqz
        br_if $B104
        get_local $l14
        get_local $p3
        i32.const 16
        i32.shl
        i32.const 16
        i32.shr_s
        get_local $l29
        i32.const 26
        i32.eq
        get_local $p0
        i32.const 2632
        i32.add
        i32.load
        call_indirect (type $t5)
        br $B104
      end
      get_local $l18
      if $I106
        block $B107
          get_local $p3
          i32.const 2
          i32.ne
          br_if $B107
          get_local $p0
          i32.load offset=200
          i32.load offset=13096
          i32.eqz
          br_if $B107
          get_local $l6
          i32.load offset=31244
          i32.const 1
          i32.ne
          br_if $B107
          i32.const 0
          set_local $l0
          loop $L108
            get_local $l14
            i32.const 15
            get_local $l0
            i32.sub
            i32.const 1
            i32.shl
            i32.add
            tee_local $p4
            i32.load16_u
            set_local $p5
            get_local $p4
            get_local $l14
            get_local $l0
            i32.const 1
            i32.shl
            i32.add
            tee_local $p4
            i32.load16_u
            i32.store16
            get_local $p4
            get_local $p5
            i32.store16
            get_local $l0
            i32.const 1
            i32.add
            tee_local $l0
            i32.const 8
            i32.ne
            br_if $L108
          end
        end
        get_local $l14
        get_local $p3
        i32.const 16
        i32.shl
        i32.const 16
        i32.shr_s
        get_local $p0
        i32.const 2628
        i32.add
        i32.load
        call_indirect (type $t4)
        get_local $p0
        i32.load offset=200
        i32.load offset=13104
        i32.eqz
        get_local $l39
        i32.const 10
        i32.ne
        i32.or
        br_if $B104
        get_local $l6
        i32.load offset=31244
        i32.const 1
        i32.ne
        br_if $B104
        get_local $l14
        get_local $p3
        i32.const 16
        i32.shl
        i32.const 16
        i32.shr_s
        get_local $l29
        i32.const 26
        i32.eq
        get_local $p0
        i32.const 2632
        i32.add
        i32.load
        call_indirect (type $t5)
        br $B104
      end
      block $B109
        get_local $p3
        i32.const 2
        i32.ne
        get_local $p5
        i32.or
        br_if $B109
        get_local $l6
        i32.load offset=31244
        i32.const 1
        i32.ne
        br_if $B109
        get_local $l14
        get_local $p0
        i32.const 2636
        i32.add
        i32.load
        call_indirect (type $t1)
        br $B104
      end
      get_local $l7
      get_local $l3
      get_local $l3
      get_local $l7
      i32.lt_s
      select
      tee_local $p3
      i32.eqz
      if $I110
        get_local $l14
        get_local $p0
        get_local $l30
        i32.const 2
        i32.shl
        i32.add
        i32.const 2656
        i32.add
        i32.load
        call_indirect (type $t1)
        br $B104
      end
      get_local $l3
      get_local $l7
      i32.add
      tee_local $p4
      i32.const 4
      i32.add
      set_local $l0
      block $B111
        get_local $p3
        i32.const 3
        i32.le_s
        if $I112
          get_local $l0
          i32.const 4
          get_local $p4
          i32.const 0
          i32.lt_s
          select
          set_local $l0
          br $B111
        end
        get_local $p3
        i32.const 7
        i32.le_s
        if $I113
          get_local $l0
          i32.const 8
          get_local $l0
          i32.const 8
          i32.lt_s
          select
          set_local $l0
          br $B111
        end
        get_local $p3
        i32.const 11
        i32.gt_s
        br_if $B111
        get_local $l0
        i32.const 24
        get_local $l0
        i32.const 24
        i32.lt_s
        select
        set_local $l0
      end
      get_local $l14
      get_local $l0
      get_local $p0
      get_local $l30
      i32.const 2
      i32.shl
      i32.add
      i32.const 2640
      i32.add
      i32.load
      call_indirect (type $t4)
    end
    get_local $l6
    i32.load8_u offset=304
    i32.eqz
    get_local $l32
    i32.const 1
    i32.lt_s
    i32.or
    i32.eqz
    if $I114
      get_local $l6
      i32.load offset=284
      set_local $p3
      i32.const 0
      set_local $l0
      loop $L115
        get_local $l14
        get_local $l0
        i32.const 1
        i32.shl
        tee_local $p4
        i32.add
        tee_local $p5
        get_local $p5
        i32.load16_u
        get_local $p3
        get_local $p4
        get_local $l46
        i32.add
        i32.load16_s
        i32.mul
        i32.const 3
        i32.shr_u
        i32.add
        i32.store16
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $l32
        i32.ne
        br_if $L115
      end
    end
    get_local $l42
    get_local $p2
    get_local $l43
    i32.shr_s
    get_local $l31
    i32.mul
    get_local $p1
    get_local $l44
    i32.shr_s
    get_local $l45
    i32.shl
    i32.add
    i32.add
    get_local $l14
    get_local $l31
    get_local $p0
    get_local $l30
    i32.const 2
    i32.shl
    i32.add
    i32.const 2612
    i32.add
    i32.load
    call_indirect (type $t5)
    get_local $l11
    i32.const 96
    i32.add
    set_global $g0)
  (func $ff_hevc_set_qPy (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32)
    get_local $p0
    get_local $p1
    get_local $p2
    call $get_qPy_pred
    set_local $p1
    get_local $p0
    i32.load offset=136
    tee_local $p2
    i32.load offset=280
    tee_local $l0
    if $I0
      get_local $p1
      get_local $l0
      i32.add
      get_local $p0
      i32.load offset=200
      i32.load offset=13192
      tee_local $p0
      i32.const 1
      i32.shl
      i32.add
      i32.const 52
      i32.add
      tee_local $p1
      i32.const 0
      i32.const -51
      get_local $p0
      i32.sub
      get_local $p1
      i32.const 0
      i32.gt_s
      select
      i32.add
      tee_local $l0
      get_local $p0
      i32.const 52
      i32.add
      i32.rem_s
      get_local $p1
      i32.add
      get_local $p0
      get_local $l0
      i32.add
      i32.sub
      set_local $p1
    end
    get_local $p2
    get_local $p1
    i32.store8 offset=272)
  (func $get_qPy_pred (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32)
    i32.const -1
    get_local $p0
    i32.load offset=200
    tee_local $l1
    i32.load offset=13080
    tee_local $l0
    get_local $p0
    i32.load offset=204
    i32.load offset=24
    i32.sub
    i32.shl
    tee_local $l2
    get_local $p2
    i32.and
    set_local $l5
    get_local $p1
    get_local $l2
    i32.and
    set_local $l6
    i32.const -1
    get_local $l0
    i32.shl
    i32.const -1
    i32.xor
    tee_local $l4
    get_local $p1
    i32.and
    set_local $l7
    get_local $l1
    i32.load offset=13064
    set_local $l3
    get_local $l1
    i32.load offset=13140
    set_local $l1
    block $B0 (result i32)
      get_local $p0
      i32.load offset=136
      tee_local $l0
      i32.load8_u offset=203
      i32.eqz
      i32.const 0
      get_local $l2
      get_local $p1
      get_local $p2
      i32.or
      i32.and
      select
      i32.eqz
      if $I1
        get_local $l0
        get_local $l0
        i32.load8_u offset=300
        i32.eqz
        i32.store8 offset=203
        get_local $p0
        i32.const 2112
        i32.add
        i32.load8_s
        br $B0
      end
      get_local $l0
      i32.load offset=276
    end
    set_local $p1
    get_local $p2
    get_local $l4
    i32.and
    set_local $l0
    get_local $l5
    get_local $l3
    i32.shr_s
    set_local $l2
    get_local $l6
    get_local $l3
    i32.shr_s
    set_local $l3
    block $B2 (result i32)
      get_local $p1
      get_local $l7
      i32.eqz
      br_if $B2
      drop
      get_local $p1
      get_local $l4
      get_local $l6
      i32.and
      i32.eqz
      br_if $B2
      drop
      get_local $p0
      i32.load offset=4316
      get_local $l3
      get_local $l1
      get_local $l2
      i32.mul
      i32.add
      i32.add
      i32.const 1
      i32.sub
      i32.load8_s
    end
    set_local $p2
    get_local $l0
    i32.eqz
    get_local $l4
    get_local $l5
    i32.and
    i32.eqz
    i32.or
    if $I3 (result i32)
      get_local $p1
    else
      get_local $p0
      i32.load offset=4316
      get_local $l2
      i32.const 1
      i32.sub
      get_local $l1
      i32.mul
      get_local $l3
      i32.add
      i32.add
      i32.load8_s
    end
    get_local $p2
    i32.add
    i32.const 1
    i32.add
    i32.const 1
    i32.shr_s)
  (func $ff_hevc_deblocking_boundary_strengths (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_local $p0
    i32.load offset=136
    set_local $l1
    block $B0
      get_local $p2
      i32.const 7
      i32.and
      get_local $p2
      i32.const 1
      i32.lt_s
      i32.or
      br_if $B0
      get_local $p0
      i32.load offset=200
      set_local $l0
      block $B1
        get_local $p0
        i32.const 2062
        i32.add
        i32.load8_u
        br_if $B1
        get_local $l1
        i32.load8_u offset=31312
        i32.const 4
        i32.and
        i32.eqz
        br_if $B1
        i32.const -1
        get_local $l0
        i32.load offset=13080
        i32.shl
        i32.const -1
        i32.xor
        get_local $p2
        i32.and
        i32.eqz
        br_if $B0
      end
      block $B2
        get_local $p0
        i32.load offset=204
        i32.load8_u offset=53
        br_if $B2
        get_local $l1
        i32.load8_u offset=31312
        i32.const 8
        i32.and
        i32.eqz
        br_if $B2
        i32.const -1
        get_local $l0
        i32.load offset=13080
        i32.shl
        i32.const -1
        i32.xor
        get_local $p2
        i32.and
        i32.eqz
        br_if $B0
      end
      get_local $p3
      i32.const 31
      i32.eq
      br_if $B0
      i32.const 1
      get_local $p3
      i32.shl
      set_local $l2
      i32.const 0
      set_local $l0
      loop $L3
        get_local $p0
        i32.load offset=4320
        get_local $p1
        get_local $l0
        i32.add
        get_local $p0
        i32.load offset=2596
        get_local $p2
        i32.mul
        i32.add
        i32.const 2
        i32.shr_s
        i32.add
        i32.const 2
        i32.store8
        get_local $l0
        i32.const 4
        i32.add
        tee_local $l0
        get_local $l2
        i32.lt_s
        br_if $L3
      end
    end
    block $B4
      get_local $p1
      i32.const 7
      i32.and
      get_local $p1
      i32.const 1
      i32.lt_s
      i32.or
      br_if $B4
      block $B5
        get_local $p0
        i32.const 2062
        i32.add
        i32.load8_u
        br_if $B5
        get_local $l1
        i32.load8_u offset=31312
        i32.const 1
        i32.and
        i32.eqz
        br_if $B5
        i32.const -1
        get_local $p0
        i32.load offset=200
        i32.load offset=13080
        i32.shl
        i32.const -1
        i32.xor
        get_local $p1
        i32.and
        i32.eqz
        br_if $B4
      end
      block $B6
        get_local $p0
        i32.load offset=204
        i32.load8_u offset=53
        br_if $B6
        get_local $l1
        i32.load8_u offset=31312
        i32.const 2
        i32.and
        i32.eqz
        br_if $B6
        i32.const -1
        get_local $p0
        i32.load offset=200
        i32.load offset=13080
        i32.shl
        i32.const -1
        i32.xor
        get_local $p1
        i32.and
        i32.eqz
        br_if $B4
      end
      get_local $p3
      i32.const 31
      i32.eq
      br_if $B4
      i32.const 1
      get_local $p3
      i32.shl
      set_local $p3
      i32.const 0
      set_local $l0
      loop $L7
        get_local $p0
        i32.load offset=4324
        get_local $p0
        i32.load offset=2596
        get_local $p2
        get_local $l0
        i32.add
        i32.mul
        get_local $p1
        i32.add
        i32.const 2
        i32.shr_s
        i32.add
        i32.const 2
        i32.store8
        get_local $l0
        i32.const 4
        i32.add
        tee_local $l0
        get_local $p3
        i32.lt_s
        br_if $L7
      end
    end)
  (func $ff_hevc_hls_filter (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load offset=200
    i32.load offset=13120
    set_local $l0
    get_local $p0
    get_local $p1
    get_local $p2
    call $deblocking_filter_CTB
    get_local $l0
    get_local $p3
    i32.sub
    set_local $l0
    block $B0
      get_local $p0
      i32.load offset=200
      tee_local $l1
      i32.load8_u offset=12941
      if $I1
        get_local $l1
        i32.load offset=13124
        set_local $l1
        get_local $p1
        i32.eqz
        get_local $p2
        i32.eqz
        i32.or
        i32.eqz
        if $I2
          get_local $p0
          get_local $p1
          get_local $p3
          i32.sub
          get_local $p2
          get_local $p3
          i32.sub
          call $sao_filter_CTB
        end
        get_local $p1
        i32.eqz
        get_local $l1
        get_local $p3
        i32.sub
        tee_local $l1
        get_local $p2
        i32.gt_s
        i32.or
        i32.eqz
        if $I3
          get_local $p0
          get_local $p1
          get_local $p3
          i32.sub
          get_local $p2
          call $sao_filter_CTB
        end
        block $B4
          get_local $p2
          i32.eqz
          get_local $p1
          get_local $l0
          i32.lt_s
          i32.or
          br_if $B4
          get_local $p0
          get_local $p1
          get_local $p2
          get_local $p3
          i32.sub
          call $sao_filter_CTB
          get_local $p0
          i32.load8_u offset=140
          i32.const 1
          i32.and
          i32.eqz
          br_if $B4
          get_local $p0
          i32.load offset=2520
          drop
        end
        get_local $p1
        get_local $l0
        i32.lt_s
        get_local $p2
        get_local $l1
        i32.lt_s
        i32.or
        br_if $B0
        get_local $p0
        get_local $p1
        get_local $p2
        call $sao_filter_CTB
        get_local $p0
        i32.load8_u offset=140
        i32.const 1
        i32.and
        i32.eqz
        br_if $B0
        get_local $p0
        i32.load offset=2520
        drop
        return
      end
      get_local $p1
      get_local $l0
      i32.lt_s
      br_if $B0
      get_local $p0
      i32.load8_u offset=140
      i32.const 1
      i32.and
      i32.eqz
      br_if $B0
      get_local $p0
      i32.load offset=2520
      drop
    end)
  (func $deblocking_filter_CTB (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32) (local $l13 i32) (local $l14 i32) (local $l15 i32) (local $l16 i32) (local $l17 i32) (local $l18 i32) (local $l19 i32) (local $l20 i32) (local $l21 i32) (local $l22 i32) (local $l23 i32) (local $l24 i32) (local $l25 i32)
    get_global $g0
    i32.const 32
    i32.sub
    tee_local $l1
    set_global $g0
    get_local $l1
    i32.const 0
    i32.store16 offset=14
    get_local $l1
    i32.const 0
    i32.store16 offset=12
    get_local $p0
    i32.load offset=2508
    tee_local $l18
    get_local $p2
    get_local $p0
    i32.load offset=200
    tee_local $l0
    i32.load offset=13080
    tee_local $l5
    i32.shr_s
    get_local $l0
    i32.load offset=13128
    i32.mul
    get_local $p1
    get_local $l5
    i32.shr_s
    i32.add
    tee_local $l12
    i32.const 3
    i32.shl
    i32.add
    set_local $l3
    block $B0 (result i32)
      get_local $l0
      i32.load offset=68
      if $I1
        i32.const 1
        get_local $l0
        i32.const 13056
        i32.add
        i32.load8_u
        br_if $B0
        drop
      end
      get_local $p0
      i32.load offset=204
      i32.load8_u offset=40
      i32.const 0
      i32.ne
    end
    set_local $l23
    get_local $l3
    i32.load offset=4
    set_local $l19
    i32.const 1
    get_local $l5
    i32.shl
    set_local $l2
    block $B2 (result i32)
      get_local $p1
      i32.eqz
      if $I3
        get_local $l19
        set_local $l18
        i32.const 0
        br $B2
      end
      get_local $l12
      i32.const 3
      i32.shl
      get_local $l18
      i32.add
      i32.const 8
      i32.sub
      tee_local $l5
      i32.load
      set_local $l15
      get_local $l5
      i32.load offset=4
      tee_local $l18
    end
    set_local $l20
    get_local $l0
    i32.load offset=13120
    tee_local $l12
    get_local $p1
    get_local $l2
    i32.add
    tee_local $l13
    get_local $l12
    get_local $l13
    i32.lt_s
    select
    set_local $l14
    get_local $l19
    set_local $l5
    get_local $l0
    i32.load offset=13124
    tee_local $l4
    get_local $p2
    get_local $l2
    i32.add
    tee_local $l2
    get_local $l2
    get_local $l4
    i32.gt_s
    select
    tee_local $l24
    get_local $p2
    i32.le_s
    tee_local $l25
    i32.eqz
    if $I4
      get_local $l14
      i32.const 8
      i32.sub
      get_local $l14
      get_local $l12
      get_local $l13
      i32.gt_s
      select
      set_local $l7
      get_local $p1
      i32.const 8
      i32.sub
      i32.const 0
      get_local $p1
      select
      set_local $l12
      get_local $p1
      i32.const 8
      get_local $p1
      select
      set_local $l13
      get_local $p2
      set_local $l2
      get_local $l3
      i32.load
      tee_local $l21
      set_local $l9
      loop $L5
        get_local $l13
        get_local $l14
        i32.lt_s
        if $I6
          get_local $l5
          i32.const -2
          i32.and
          set_local $l8
          get_local $l2
          i32.const 4
          i32.add
          set_local $l4
          get_local $l13
          set_local $l0
          loop $L7
            block $B8
              get_local $p0
              i32.load offset=4324
              tee_local $l3
              get_local $p0
              i32.load offset=2596
              tee_local $l6
              get_local $l4
              i32.mul
              get_local $l0
              i32.add
              i32.const 2
              i32.shr_s
              i32.add
              i32.load8_u
              tee_local $l10
              get_local $l3
              get_local $l2
              get_local $l6
              i32.mul
              get_local $l0
              i32.add
              i32.const 2
              i32.shr_s
              i32.add
              i32.load8_u
              tee_local $l6
              i32.or
              i32.eqz
              br_if $B8
              i32.const 0
              set_local $l3
              get_local $p0
              get_local $l0
              i32.const 1
              i32.sub
              tee_local $l16
              get_local $l2
              call $get_qPy
              get_local $p0
              get_local $l0
              get_local $l2
              call $get_qPy
              i32.add
              i32.const 1
              i32.add
              i32.const 1
              i32.shr_s
              tee_local $l17
              get_local $l9
              i32.add
              tee_local $l11
              i32.const 51
              get_local $l11
              i32.const 51
              i32.lt_s
              select
              tee_local $l11
              i32.const 0
              get_local $l11
              i32.const 0
              i32.gt_s
              select
              set_local $l22
              i32.const 0
              set_local $l11
              get_local $l1
              get_local $l6
              if $I9 (result i32)
                get_local $l8
                get_local $l17
                i32.add
                get_local $l6
                i32.const 1
                i32.shl
                i32.add
                i32.const 2
                i32.sub
                tee_local $l6
                i32.const 53
                get_local $l6
                i32.const 53
                i32.lt_s
                select
                tee_local $l6
                i32.const 0
                get_local $l6
                i32.const 0
                i32.gt_s
                select
                i32.const 2384
                i32.add
                i32.load8_u
              else
                get_local $l11
              end
              i32.store offset=16
              get_local $l10
              if $I10
                get_local $l8
                get_local $l17
                i32.add
                get_local $l10
                i32.const 1
                i32.shl
                i32.add
                i32.const 2
                i32.sub
                tee_local $l3
                i32.const 53
                get_local $l3
                i32.const 53
                i32.lt_s
                select
                tee_local $l3
                i32.const 0
                get_local $l3
                i32.const 0
                i32.gt_s
                select
                i32.const 2384
                i32.add
                i32.load8_u
                set_local $l3
              end
              get_local $l22
              i32.const 2320
              i32.add
              i32.load8_u
              set_local $l10
              get_local $l1
              get_local $l3
              i32.store offset=20
              get_local $p0
              i32.load offset=160
              tee_local $l3
              i32.load
              get_local $l3
              i32.load offset=32
              tee_local $l3
              get_local $l2
              i32.mul
              get_local $l0
              get_local $p0
              i32.load offset=200
              i32.load offset=56
              i32.shl
              i32.add
              i32.add
              set_local $l6
              get_local $l23
              if $I11
                get_local $l1
                get_local $p0
                get_local $l16
                get_local $l2
                call $get_pcm
                i32.store8 offset=14
                get_local $l1
                get_local $p0
                get_local $l16
                get_local $l4
                call $get_pcm
                i32.store8 offset=15
                get_local $l1
                get_local $p0
                get_local $l0
                get_local $l2
                call $get_pcm
                i32.store8 offset=12
                get_local $l1
                get_local $p0
                get_local $l0
                get_local $l4
                call $get_pcm
                i32.store8 offset=13
                get_local $l6
                get_local $l3
                get_local $l10
                get_local $l1
                i32.const 16
                i32.add
                get_local $l1
                i32.const 14
                i32.add
                get_local $l1
                i32.const 12
                i32.add
                get_local $p0
                i32.load offset=4304
                call_indirect (type $t6)
                br $B8
              end
              get_local $l6
              get_local $l3
              get_local $l10
              get_local $l1
              i32.const 16
              i32.add
              get_local $l1
              i32.const 14
              i32.add
              get_local $l1
              i32.const 12
              i32.add
              get_local $p0
              i32.load offset=4288
              call_indirect (type $t6)
            end
            get_local $l0
            i32.const 8
            i32.add
            tee_local $l0
            get_local $l14
            i32.lt_s
            br_if $L7
          end
        end
        get_local $l2
        i32.eqz
        get_local $l7
        get_local $l12
        i32.le_s
        i32.or
        i32.eqz
        if $I12
          get_local $l2
          i32.const 1
          i32.sub
          set_local $l8
          get_local $l12
          set_local $l0
          loop $L13
            block $B14
              get_local $p0
              i32.load offset=4320
              tee_local $l3
              get_local $p0
              i32.load offset=2596
              get_local $l2
              i32.mul
              tee_local $l4
              get_local $l0
              i32.const 4
              i32.add
              tee_local $l10
              i32.add
              i32.const 2
              i32.shr_s
              i32.add
              i32.load8_u
              tee_local $l6
              get_local $l3
              get_local $l0
              get_local $l4
              i32.add
              i32.const 2
              i32.shr_s
              i32.add
              i32.load8_u
              tee_local $l16
              i32.or
              i32.eqz
              br_if $B14
              i32.const 0
              set_local $l3
              get_local $p0
              get_local $l0
              get_local $l8
              call $get_qPy
              get_local $p0
              get_local $l0
              get_local $l2
              call $get_qPy
              i32.add
              i32.const 1
              i32.add
              i32.const 1
              i32.shr_s
              tee_local $l17
              get_local $l15
              get_local $l21
              get_local $p1
              get_local $l0
              i32.gt_s
              tee_local $l5
              select
              tee_local $l9
              i32.add
              tee_local $l4
              i32.const 51
              get_local $l4
              i32.const 51
              i32.lt_s
              select
              tee_local $l4
              i32.const 0
              get_local $l4
              i32.const 0
              i32.gt_s
              select
              set_local $l11
              get_local $l20
              get_local $l19
              get_local $l5
              select
              set_local $l5
              i32.const 0
              set_local $l4
              get_local $l1
              get_local $l16
              if $I15 (result i32)
                get_local $l17
                get_local $l5
                i32.const -2
                i32.and
                i32.add
                get_local $l16
                i32.const 1
                i32.shl
                i32.add
                i32.const 2
                i32.sub
                tee_local $l4
                i32.const 53
                get_local $l4
                i32.const 53
                i32.lt_s
                select
                tee_local $l4
                i32.const 0
                get_local $l4
                i32.const 0
                i32.gt_s
                select
                i32.const 2384
                i32.add
                i32.load8_u
              else
                get_local $l4
              end
              i32.store offset=16
              get_local $l11
              i32.const 2320
              i32.add
              i32.load8_u
              set_local $l4
              get_local $l1
              get_local $l6
              if $I16 (result i32)
                get_local $l17
                get_local $l5
                i32.const -2
                i32.and
                i32.add
                get_local $l6
                i32.const 1
                i32.shl
                i32.add
                i32.const 2
                i32.sub
                tee_local $l3
                i32.const 53
                get_local $l3
                i32.const 53
                i32.lt_s
                select
                tee_local $l3
                i32.const 0
                get_local $l3
                i32.const 0
                i32.gt_s
                select
                i32.const 2384
                i32.add
                i32.load8_u
              else
                get_local $l3
              end
              i32.store offset=20
              get_local $p0
              i32.load offset=160
              tee_local $l3
              i32.load
              get_local $l3
              i32.load offset=32
              tee_local $l3
              get_local $l2
              i32.mul
              get_local $l0
              get_local $p0
              i32.load offset=200
              i32.load offset=56
              i32.shl
              i32.add
              i32.add
              set_local $l6
              get_local $l23
              if $I17
                get_local $l1
                get_local $p0
                get_local $l0
                get_local $l8
                call $get_pcm
                i32.store8 offset=14
                get_local $l1
                get_local $p0
                get_local $l10
                get_local $l8
                call $get_pcm
                i32.store8 offset=15
                get_local $l1
                get_local $p0
                get_local $l0
                get_local $l2
                call $get_pcm
                i32.store8 offset=12
                get_local $l1
                get_local $p0
                get_local $l10
                get_local $l2
                call $get_pcm
                i32.store8 offset=13
                get_local $l6
                get_local $l3
                get_local $l4
                get_local $l1
                i32.const 16
                i32.add
                get_local $l1
                i32.const 14
                i32.add
                get_local $l1
                i32.const 12
                i32.add
                get_local $p0
                i32.load offset=4300
                call_indirect (type $t6)
                br $B14
              end
              get_local $l6
              get_local $l3
              get_local $l4
              get_local $l1
              i32.const 16
              i32.add
              get_local $l1
              i32.const 14
              i32.add
              get_local $l1
              i32.const 12
              i32.add
              get_local $p0
              i32.load offset=4284
              call_indirect (type $t6)
            end
            get_local $l0
            i32.const 8
            i32.add
            tee_local $l0
            get_local $l7
            i32.lt_s
            br_if $L13
          end
        end
        get_local $l2
        i32.const 8
        i32.add
        tee_local $l2
        get_local $l24
        i32.lt_s
        br_if $L5
      end
      get_local $p0
      i32.load offset=200
      set_local $l0
    end
    block $B18
      get_local $l0
      i32.load offset=4
      i32.eqz
      br_if $B18
      i32.const 1
      set_local $l3
      loop $L19
        get_local $l25
        i32.eqz
        if $I20
          get_local $p1
          i32.const 1
          get_local $l0
          get_local $l3
          i32.const 2
          i32.shl
          tee_local $l20
          i32.add
          tee_local $l0
          i32.const 13168
          i32.add
          i32.load
          i32.shl
          tee_local $l2
          i32.const 3
          i32.shl
          tee_local $l15
          i32.sub
          i32.const 0
          get_local $p1
          select
          set_local $l12
          i32.const 1
          get_local $l0
          i32.const 13180
          i32.add
          i32.load
          i32.shl
          tee_local $l0
          i32.const 3
          i32.shl
          set_local $l6
          get_local $l2
          i32.const 2
          i32.shl
          set_local $l16
          get_local $l0
          i32.const 2
          i32.shl
          set_local $l17
          get_local $p1
          get_local $l15
          get_local $p1
          select
          set_local $l13
          get_local $p2
          set_local $l2
          loop $L21
            get_local $l13
            get_local $l14
            i32.lt_s
            if $I22
              get_local $l2
              get_local $l17
              i32.add
              set_local $l8
              get_local $l13
              set_local $l0
              loop $L23
                get_local $p0
                i32.load offset=4324
                tee_local $l4
                get_local $p0
                i32.load offset=2596
                tee_local $l7
                get_local $l8
                i32.mul
                get_local $l0
                i32.add
                i32.const 2
                i32.shr_s
                i32.add
                i32.load8_u
                set_local $l9
                block $B24
                  get_local $l4
                  get_local $l2
                  get_local $l7
                  i32.mul
                  get_local $l0
                  i32.add
                  i32.const 2
                  i32.shr_s
                  i32.add
                  i32.load8_u
                  tee_local $l11
                  i32.const 2
                  i32.ne
                  i32.const 0
                  get_local $l9
                  i32.const 2
                  i32.ne
                  select
                  br_if $B24
                  get_local $p0
                  get_local $l0
                  i32.const 1
                  i32.sub
                  tee_local $l7
                  get_local $l8
                  call $get_qPy
                  set_local $l21
                  get_local $p0
                  get_local $l0
                  get_local $l8
                  call $get_qPy
                  set_local $l22
                  i32.const 0
                  set_local $l4
                  i32.const 0
                  set_local $l10
                  get_local $l1
                  get_local $l11
                  i32.const 2
                  i32.eq
                  if $I25 (result i32)
                    get_local $p0
                    get_local $p0
                    get_local $l0
                    get_local $l2
                    call $get_qPy
                    get_local $p0
                    get_local $l7
                    get_local $l2
                    call $get_qPy
                    i32.add
                    i32.const 1
                    i32.add
                    i32.const 1
                    i32.shr_s
                    get_local $l3
                    get_local $l5
                    call $chroma_tc
                  else
                    get_local $l10
                  end
                  i32.store offset=24
                  get_local $l1
                  get_local $l9
                  i32.const 2
                  i32.eq
                  if $I26 (result i32)
                    get_local $p0
                    get_local $l21
                    get_local $l22
                    i32.add
                    i32.const 1
                    i32.add
                    i32.const 1
                    i32.shr_s
                    get_local $l3
                    get_local $l5
                    call $chroma_tc
                  else
                    get_local $l4
                  end
                  i32.store offset=28
                  get_local $p0
                  i32.load offset=160
                  get_local $l20
                  i32.add
                  tee_local $l4
                  i32.load
                  get_local $l0
                  get_local $p0
                  i32.load offset=200
                  tee_local $l9
                  get_local $l20
                  i32.add
                  tee_local $l10
                  i32.const 13168
                  i32.add
                  i32.load
                  i32.shr_s
                  get_local $l9
                  i32.load offset=56
                  i32.shl
                  get_local $l4
                  i32.load offset=32
                  tee_local $l4
                  get_local $l2
                  get_local $l10
                  i32.const 13180
                  i32.add
                  i32.load
                  i32.shr_s
                  i32.mul
                  i32.add
                  i32.add
                  set_local $l9
                  get_local $l23
                  if $I27
                    get_local $l1
                    get_local $p0
                    get_local $l7
                    get_local $l2
                    call $get_pcm
                    i32.store8 offset=14
                    get_local $l1
                    get_local $p0
                    get_local $l7
                    get_local $l8
                    call $get_pcm
                    i32.store8 offset=15
                    get_local $l1
                    get_local $p0
                    get_local $l0
                    get_local $l2
                    call $get_pcm
                    i32.store8 offset=12
                    get_local $l1
                    get_local $p0
                    get_local $l0
                    get_local $l8
                    call $get_pcm
                    i32.store8 offset=13
                    get_local $l9
                    get_local $l4
                    get_local $l1
                    i32.const 24
                    i32.add
                    get_local $l1
                    i32.const 14
                    i32.add
                    get_local $l1
                    i32.const 12
                    i32.add
                    get_local $p0
                    i32.load offset=4312
                    call_indirect (type $t8)
                    br $B24
                  end
                  get_local $l9
                  get_local $l4
                  get_local $l1
                  i32.const 24
                  i32.add
                  get_local $l1
                  i32.const 14
                  i32.add
                  get_local $l1
                  i32.const 12
                  i32.add
                  get_local $p0
                  i32.load offset=4296
                  call_indirect (type $t8)
                end
                get_local $l0
                get_local $l15
                i32.add
                tee_local $l0
                get_local $l14
                i32.lt_s
                br_if $L23
              end
            end
            block $B28
              get_local $l2
              i32.eqz
              br_if $B28
              get_local $l18
              set_local $l5
              get_local $l12
              get_local $l14
              i32.const 0
              get_local $l15
              get_local $l14
              get_local $p0
              i32.load offset=200
              i32.load offset=13120
              i32.eq
              select
              i32.sub
              tee_local $l11
              i32.ge_s
              br_if $B28
              get_local $l2
              i32.const 1
              i32.sub
              set_local $l4
              get_local $l12
              set_local $l0
              loop $L29
                get_local $p0
                i32.load offset=4320
                tee_local $l5
                get_local $p0
                i32.load offset=2596
                get_local $l2
                i32.mul
                tee_local $l9
                get_local $l0
                get_local $l16
                i32.add
                tee_local $l8
                i32.add
                i32.const 2
                i32.shr_s
                i32.add
                i32.load8_u
                set_local $l7
                block $B30
                  get_local $l5
                  get_local $l0
                  get_local $l9
                  i32.add
                  i32.const 2
                  i32.shr_s
                  i32.add
                  i32.load8_u
                  tee_local $l10
                  i32.const 2
                  i32.ne
                  i32.const 0
                  get_local $l7
                  i32.const 2
                  i32.ne
                  select
                  br_if $B30
                  i32.const 0
                  set_local $l9
                  i32.const 0
                  set_local $l5
                  get_local $l10
                  i32.const 2
                  i32.ne
                  tee_local $l21
                  i32.eqz
                  if $I31
                    get_local $p0
                    get_local $l0
                    get_local $l4
                    call $get_qPy
                    get_local $p0
                    get_local $l0
                    get_local $l2
                    call $get_qPy
                    i32.add
                    i32.const 1
                    i32.add
                    i32.const 1
                    i32.shr_s
                    set_local $l5
                  end
                  get_local $l7
                  i32.const 2
                  i32.ne
                  tee_local $l22
                  i32.eqz
                  if $I32
                    get_local $p0
                    get_local $l8
                    get_local $l4
                    call $get_qPy
                    get_local $p0
                    get_local $l8
                    get_local $l2
                    call $get_qPy
                    i32.add
                    i32.const 1
                    i32.add
                    i32.const 1
                    i32.shr_s
                    set_local $l9
                  end
                  i32.const 0
                  set_local $l7
                  i32.const 0
                  set_local $l10
                  get_local $l1
                  get_local $l21
                  if $I33 (result i32)
                    get_local $l10
                  else
                    get_local $p0
                    get_local $l5
                    get_local $l3
                    get_local $l18
                    call $chroma_tc
                  end
                  i32.store offset=24
                  get_local $l1
                  get_local $l22
                  if $I34 (result i32)
                    get_local $l7
                  else
                    get_local $p0
                    get_local $l9
                    get_local $l3
                    get_local $l19
                    call $chroma_tc
                  end
                  i32.store offset=28
                  get_local $p0
                  i32.load offset=160
                  get_local $l20
                  i32.add
                  tee_local $l7
                  i32.load
                  get_local $l0
                  get_local $p0
                  i32.load offset=200
                  tee_local $l5
                  i32.const 13172
                  i32.add
                  i32.load
                  i32.shr_s
                  get_local $l5
                  i32.load offset=56
                  i32.shl
                  get_local $l7
                  i32.load offset=32
                  tee_local $l7
                  get_local $l2
                  get_local $l5
                  i32.const 13184
                  i32.add
                  i32.load
                  i32.shr_s
                  i32.mul
                  i32.add
                  i32.add
                  set_local $l5
                  get_local $l23
                  if $I35
                    get_local $l1
                    get_local $p0
                    get_local $l0
                    get_local $l4
                    call $get_pcm
                    i32.store8 offset=14
                    get_local $l1
                    get_local $p0
                    get_local $l8
                    get_local $l4
                    call $get_pcm
                    i32.store8 offset=15
                    get_local $l1
                    get_local $p0
                    get_local $l0
                    get_local $l2
                    call $get_pcm
                    i32.store8 offset=12
                    get_local $l1
                    get_local $p0
                    get_local $l8
                    get_local $l2
                    call $get_pcm
                    i32.store8 offset=13
                    get_local $l5
                    get_local $l7
                    get_local $l1
                    i32.const 24
                    i32.add
                    get_local $l1
                    i32.const 14
                    i32.add
                    get_local $l1
                    i32.const 12
                    i32.add
                    get_local $p0
                    i32.load offset=4308
                    call_indirect (type $t8)
                    br $B30
                  end
                  get_local $l5
                  get_local $l7
                  get_local $l1
                  i32.const 24
                  i32.add
                  get_local $l1
                  i32.const 14
                  i32.add
                  get_local $l1
                  i32.const 12
                  i32.add
                  get_local $p0
                  i32.load offset=4292
                  call_indirect (type $t8)
                end
                get_local $l0
                get_local $l15
                i32.add
                tee_local $l0
                get_local $l11
                i32.lt_s
                br_if $L29
              end
              get_local $l18
              set_local $l5
            end
            get_local $l2
            get_local $l6
            i32.add
            tee_local $l2
            get_local $l24
            i32.lt_s
            br_if $L21
          end
        end
        get_local $l3
        i32.const 1
        i32.add
        tee_local $l3
        i32.const 3
        i32.eq
        br_if $B18
        get_local $p0
        i32.load offset=200
        set_local $l0
        br $L19
      end
      unreachable
    end
    get_local $l1
    i32.const 32
    i32.add
    set_global $g0)
  (func $sao_filter_CTB (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32) (local $l13 i32) (local $l14 i32) (local $l15 i32) (local $l16 i32) (local $l17 i32) (local $l18 i32) (local $l19 i32) (local $l20 i32) (local $l21 i32) (local $l22 i32) (local $l23 i32) (local $l24 i32) (local $l25 i32) (local $l26 i32) (local $l27 i32) (local $l28 i32) (local $l29 i32) (local $l30 i32) (local $l31 i32) (local $l32 i32) (local $l33 i32) (local $l34 i32) (local $l35 i32)
    get_global $g0
    i32.const 32
    i32.sub
    tee_local $l3
    set_global $g0
    get_local $p0
    i32.load offset=204
    tee_local $l8
    i32.load offset=1668
    tee_local $l12
    get_local $p2
    get_local $p0
    i32.load offset=200
    tee_local $l6
    i32.load offset=13080
    tee_local $l16
    i32.shr_s
    tee_local $l13
    get_local $l6
    i32.load offset=13128
    tee_local $l10
    i32.mul
    get_local $p1
    get_local $l16
    i32.shr_s
    tee_local $l14
    i32.add
    tee_local $l5
    i32.const 2
    i32.shl
    i32.add
    i32.load
    set_local $l15
    get_local $p0
    i32.load offset=2504
    set_local $l17
    get_local $l3
    i32.const 0
    i32.store16 offset=14
    get_local $l3
    i32.const 0
    i32.store16 offset=12
    get_local $l3
    i32.const 0
    i32.store offset=8
    get_local $p0
    i32.load offset=4352
    get_local $l5
    i32.add
    i32.load8_u
    set_local $l11
    get_local $l8
    i32.load8_u offset=42
    if $I0
      get_local $l8
      i32.load8_u offset=53
      i32.eqz
      set_local $l0
    end
    get_local $l5
    i32.const 148
    i32.mul
    set_local $l18
    get_local $l3
    get_local $l13
    i32.eqz
    i32.store offset=20
    get_local $l3
    get_local $l14
    i32.eqz
    i32.store offset=16
    get_local $l3
    get_local $l14
    get_local $l10
    i32.const 1
    i32.sub
    tee_local $l19
    i32.eq
    tee_local $l1
    i32.store offset=24
    get_local $l3
    get_local $l13
    get_local $l6
    i32.load offset=13132
    i32.const 1
    i32.sub
    tee_local $l20
    i32.eq
    tee_local $l21
    i32.store offset=28
    block $B1
      get_local $l11
      i32.eqz
      get_local $l0
      i32.or
      tee_local $l22
      i32.const 1
      i32.ne
      br_if $B1
      get_local $l14
      if $I2
        get_local $l0
        if $I3
          get_local $l8
          i32.load offset=1676
          tee_local $l2
          get_local $l15
          i32.const 2
          i32.shl
          i32.add
          i32.load
          get_local $l2
          get_local $l5
          i32.const 2
          i32.shl
          get_local $l12
          i32.add
          i32.const 4
          i32.sub
          i32.load
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.ne
          set_local $l9
        end
        get_local $l3
        block $B4 (result i32)
          get_local $l11
          i32.eqz
          if $I5
            i32.const 1
            get_local $p0
            i32.load offset=4328
            get_local $l5
            i32.const 2
            i32.shl
            i32.add
            tee_local $l2
            i32.load
            get_local $l2
            i32.const 4
            i32.sub
            i32.load
            i32.ne
            br_if $B4
            drop
          end
          get_local $l9
        end
        i32.store8 offset=14
      end
      get_local $l1
      i32.eqz
      if $I6
        get_local $l0
        if $I7
          get_local $l8
          i32.load offset=1676
          tee_local $l1
          get_local $l15
          i32.const 2
          i32.shl
          i32.add
          i32.load
          get_local $l1
          get_local $l5
          i32.const 2
          i32.shl
          get_local $l12
          i32.add
          i32.load offset=4
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.ne
          set_local $l4
        end
        get_local $l3
        block $B8 (result i32)
          get_local $l11
          i32.eqz
          if $I9
            i32.const 1
            get_local $p0
            i32.load offset=4328
            get_local $l5
            i32.const 2
            i32.shl
            i32.add
            tee_local $l1
            i32.load
            get_local $l1
            i32.load offset=4
            i32.ne
            br_if $B8
            drop
          end
          get_local $l4
        end
        i32.store8 offset=15
      end
      i32.const 0
      set_local $l1
      i32.const 0
      set_local $l2
      get_local $l13
      if $I10
        get_local $l0
        if $I11
          get_local $l8
          i32.load offset=1676
          tee_local $l2
          get_local $l15
          i32.const 2
          i32.shl
          i32.add
          i32.load
          get_local $l2
          get_local $l12
          get_local $l5
          get_local $l10
          i32.sub
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.ne
          set_local $l2
        end
        get_local $l3
        block $B12 (result i32)
          get_local $l11
          i32.eqz
          if $I13
            i32.const 1
            get_local $p0
            i32.load offset=4328
            tee_local $l23
            get_local $l5
            i32.const 2
            i32.shl
            i32.add
            i32.load
            get_local $l23
            get_local $l10
            get_local $l13
            i32.const 1
            i32.sub
            i32.mul
            get_local $l14
            i32.add
            i32.const 2
            i32.shl
            i32.add
            i32.load
            i32.ne
            br_if $B12
            drop
          end
          get_local $l2
        end
        i32.store8 offset=12
      end
      get_local $l21
      i32.eqz
      if $I14
        get_local $l0
        if $I15
          get_local $l8
          i32.load offset=1676
          tee_local $l1
          get_local $l15
          i32.const 2
          i32.shl
          i32.add
          i32.load
          get_local $l1
          get_local $l12
          get_local $l5
          get_local $l10
          i32.add
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.ne
          set_local $l1
        end
        get_local $l3
        block $B16 (result i32)
          get_local $l11
          i32.eqz
          if $I17
            i32.const 1
            get_local $p0
            i32.load offset=4328
            tee_local $l7
            get_local $l5
            i32.const 2
            i32.shl
            i32.add
            i32.load
            get_local $l7
            get_local $l10
            get_local $l13
            i32.const 1
            i32.add
            i32.mul
            get_local $l14
            i32.add
            i32.const 2
            i32.shl
            i32.add
            i32.load
            i32.ne
            br_if $B16
            drop
          end
          get_local $l1
        end
        i32.store8 offset=13
      end
      get_local $l14
      i32.const 0
      i32.ne
      tee_local $l7
      get_local $l13
      i32.const 0
      i32.ne
      tee_local $l8
      i32.and
      if $I18
        block $B19
          block $B20
            get_local $l11
            i32.eqz
            if $I21
              i32.const 1
              set_local $l0
              get_local $l9
              br_if $B19
              get_local $p0
              i32.load offset=4328
              tee_local $l12
              get_local $l5
              i32.const 2
              i32.shl
              i32.add
              i32.load
              get_local $l14
              get_local $l10
              get_local $l13
              i32.const 1
              i32.sub
              i32.mul
              i32.add
              i32.const 2
              i32.shl
              get_local $l12
              i32.add
              i32.const 4
              i32.sub
              i32.load
              i32.eq
              br_if $B20
              br $B19
            end
            i32.const 1
            set_local $l0
            get_local $l9
            br_if $B19
          end
          get_local $l2
          set_local $l0
        end
        get_local $l3
        get_local $l0
        i32.store8 offset=8
      end
      get_local $l8
      get_local $l14
      get_local $l19
      i32.ne
      tee_local $l8
      i32.and
      if $I22
        block $B23
          block $B24
            get_local $l11
            i32.eqz
            if $I25
              i32.const 1
              set_local $l0
              get_local $l4
              br_if $B23
              get_local $p0
              i32.load offset=4328
              tee_local $l12
              get_local $l5
              i32.const 2
              i32.shl
              i32.add
              i32.load
              get_local $l14
              get_local $l10
              get_local $l13
              i32.const 1
              i32.sub
              i32.mul
              i32.add
              i32.const 2
              i32.shl
              get_local $l12
              i32.add
              i32.load offset=4
              i32.eq
              br_if $B24
              br $B23
            end
            i32.const 1
            set_local $l0
            get_local $l4
            br_if $B23
          end
          get_local $l2
          set_local $l0
        end
        get_local $l3
        get_local $l0
        i32.store8 offset=9
      end
      get_local $l13
      get_local $l20
      i32.ne
      tee_local $l2
      get_local $l8
      i32.and
      if $I26
        block $B27
          block $B28
            get_local $l11
            i32.eqz
            if $I29
              i32.const 1
              set_local $l0
              get_local $l4
              br_if $B27
              get_local $p0
              i32.load offset=4328
              tee_local $l4
              get_local $l5
              i32.const 2
              i32.shl
              i32.add
              i32.load
              get_local $l14
              get_local $l10
              get_local $l13
              i32.const 1
              i32.add
              i32.mul
              i32.add
              i32.const 2
              i32.shl
              get_local $l4
              i32.add
              i32.load offset=4
              i32.eq
              br_if $B28
              br $B27
            end
            i32.const 1
            set_local $l0
            get_local $l4
            br_if $B27
          end
          get_local $l1
          set_local $l0
        end
        get_local $l3
        get_local $l0
        i32.store8 offset=10
      end
      get_local $l2
      get_local $l7
      i32.and
      i32.eqz
      br_if $B1
      block $B30
        block $B31
          get_local $l11
          i32.eqz
          if $I32
            i32.const 1
            set_local $l2
            get_local $l9
            br_if $B30
            get_local $p0
            i32.load offset=4328
            tee_local $l0
            get_local $l5
            i32.const 2
            i32.shl
            i32.add
            i32.load
            get_local $l14
            get_local $l10
            get_local $l13
            i32.const 1
            i32.add
            i32.mul
            i32.add
            i32.const 2
            i32.shl
            get_local $l0
            i32.add
            i32.const 4
            i32.sub
            i32.load
            i32.eq
            br_if $B31
            br $B30
          end
          i32.const 1
          set_local $l2
          get_local $l9
          br_if $B30
        end
        get_local $l1
        set_local $l2
      end
      get_local $l3
      get_local $l2
      i32.store8 offset=11
    end
    get_local $l17
    get_local $l18
    i32.add
    set_local $l24
    i32.const 3
    i32.const 1
    get_local $l6
    i32.load offset=4
    select
    set_local $l27
    get_local $l13
    i32.const 1
    i32.add
    set_local $l19
    get_local $l14
    i32.const 1
    i32.add
    set_local $l20
    get_local $l14
    i32.const 1
    i32.sub
    set_local $l21
    get_local $l13
    i32.const 1
    i32.sub
    set_local $l23
    get_local $l14
    i32.const 1
    i32.shl
    tee_local $l1
    i32.const 2
    i32.add
    set_local $l28
    get_local $l1
    i32.const 1
    i32.sub
    set_local $l29
    get_local $l13
    i32.const 1
    i32.shl
    tee_local $l1
    i32.const 2
    i32.add
    set_local $l30
    get_local $l1
    i32.const 1
    i32.sub
    set_local $l31
    get_local $p0
    get_local $l22
    i32.const 2
    i32.shl
    i32.add
    i32.const 2676
    i32.add
    set_local $l32
    i32.const 0
    set_local $l2
    get_local $l17
    get_local $l5
    i32.const 148
    i32.mul
    i32.add
    set_local $l33
    loop $L33
      get_local $l6
      i32.load offset=13124
      get_local $l6
      get_local $l2
      i32.const 2
      i32.shl
      tee_local $l15
      i32.add
      tee_local $l4
      i32.const 13180
      i32.add
      i32.load
      tee_local $l1
      i32.shr_s
      tee_local $l25
      get_local $p2
      get_local $l1
      i32.shr_s
      tee_local $l17
      i32.sub
      tee_local $l7
      i32.const 1
      get_local $l16
      i32.shl
      tee_local $l0
      get_local $l1
      i32.shr_s
      tee_local $l1
      get_local $l1
      get_local $l7
      i32.gt_s
      select
      set_local $l9
      get_local $l6
      i32.load offset=13120
      get_local $l4
      i32.const 13168
      i32.add
      i32.load
      tee_local $l1
      i32.shr_s
      tee_local $l26
      get_local $p1
      get_local $l1
      i32.shr_s
      tee_local $l16
      i32.sub
      tee_local $l4
      get_local $l0
      get_local $l1
      i32.shr_s
      tee_local $l1
      get_local $l1
      get_local $l4
      i32.gt_s
      select
      set_local $l5
      get_local $l0
      i32.const 2
      i32.add
      get_local $l6
      i32.load offset=56
      tee_local $l4
      i32.shl
      tee_local $l10
      get_local $p0
      i32.load offset=168
      i32.add
      i32.const 1
      get_local $l4
      i32.shl
      tee_local $l1
      i32.add
      set_local $l11
      get_local $p0
      i32.load offset=160
      get_local $l15
      i32.add
      tee_local $l0
      i32.load
      get_local $l0
      i32.load offset=32
      tee_local $l8
      get_local $l17
      i32.mul
      get_local $l16
      get_local $l4
      i32.shl
      i32.add
      i32.add
      set_local $l12
      block $B34
        block $B35
          block $B36
            block $B37
              get_local $l2
              get_local $l33
              i32.add
              tee_local $l34
              i32.const 142
              i32.add
              i32.load8_u
              i32.const 1
              i32.sub
              br_table $B37 $B36 $B34
            end
            get_local $l11
            get_local $l12
            get_local $l5
            get_local $l4
            i32.shl
            get_local $l9
            get_local $l10
            get_local $l8
            call $copy_CTB
            get_local $p0
            get_local $l12
            get_local $l8
            get_local $l16
            get_local $l17
            get_local $l5
            get_local $l9
            get_local $l2
            get_local $l14
            get_local $l13
            call $copy_CTB_to_hv
            get_local $l12
            get_local $l11
            get_local $l8
            get_local $l10
            get_local $l24
            get_local $l3
            i32.const 16
            i32.add
            get_local $l5
            get_local $l9
            get_local $l2
            get_local $p0
            i32.load offset=2672
            call_indirect (type $t17)
            br $B35
          end
          get_local $l3
          i32.load offset=28
          set_local $l35
          get_local $l3
          i32.load offset=24
          set_local $l22
          get_local $l3
          i32.load offset=16
          set_local $l18
          block $B38
            get_local $l3
            i32.load offset=20
            br_if $B38
            get_local $l3
            get_local $l18
            i32.const 1
            i32.sub
            tee_local $l0
            get_local $l4
            i32.shl
            tee_local $l7
            get_local $l12
            get_local $l8
            i32.sub
            i32.add
            i32.store
            get_local $l3
            get_local $p0
            get_local $l15
            i32.add
            i32.load offset=172
            get_local $l26
            get_local $l31
            i32.mul
            get_local $l16
            i32.add
            get_local $l0
            i32.add
            get_local $l4
            i32.shl
            i32.add
            i32.store offset=4
            get_local $l11
            get_local $l10
            i32.sub
            get_local $l7
            i32.add
            set_local $l7
            i32.const 0
            set_local $l0
            block $B39 (result i32)
              get_local $l18
              i32.const 1
              i32.ne
              if $I40
                get_local $l7
                get_local $l3
                get_local $p0
                i32.load offset=2504
                get_local $l21
                get_local $l6
                i32.load offset=13128
                get_local $l23
                i32.mul
                i32.add
                i32.const 148
                i32.mul
                i32.add
                get_local $l2
                i32.add
                i32.load8_u offset=142
                i32.const 3
                i32.eq
                i32.const 2
                i32.shl
                i32.add
                i32.load
                get_local $l4
                call $copy_pixel
                get_local $p0
                i32.load offset=200
                set_local $l6
                get_local $l1
                set_local $l0
              end
              get_local $l0
              get_local $l7
              i32.add
            end
            get_local $l3
            get_local $p0
            i32.load offset=2504
            get_local $l6
            i32.load offset=13128
            get_local $l23
            i32.mul
            get_local $l14
            i32.add
            i32.const 148
            i32.mul
            i32.add
            get_local $l2
            i32.add
            i32.load8_u offset=142
            i32.const 3
            i32.eq
            i32.const 2
            i32.shl
            i32.add
            i32.load
            get_local $l0
            i32.add
            get_local $l5
            get_local $l4
            i32.shl
            tee_local $l6
            call $memcpy
            drop
            get_local $l22
            i32.const 1
            i32.eq
            br_if $B38
            get_local $l7
            get_local $l0
            get_local $l6
            i32.add
            tee_local $l0
            i32.add
            get_local $l3
            get_local $p0
            i32.load offset=2504
            get_local $l20
            get_local $p0
            i32.load offset=200
            i32.load offset=13128
            get_local $l23
            i32.mul
            i32.add
            i32.const 148
            i32.mul
            i32.add
            get_local $l2
            i32.add
            i32.load8_u offset=142
            i32.const 3
            i32.eq
            i32.const 2
            i32.shl
            i32.add
            i32.load
            get_local $l0
            i32.add
            get_local $l4
            call $copy_pixel
          end
          block $B41
            get_local $l35
            br_if $B41
            get_local $l3
            get_local $l18
            i32.const 1
            i32.sub
            tee_local $l0
            get_local $l4
            i32.shl
            tee_local $l7
            get_local $l12
            get_local $l8
            get_local $l9
            i32.mul
            i32.add
            i32.add
            i32.store
            get_local $l3
            get_local $p0
            get_local $l15
            i32.add
            i32.load offset=172
            get_local $l26
            get_local $l30
            i32.mul
            get_local $l16
            i32.add
            get_local $l0
            i32.add
            get_local $l4
            i32.shl
            i32.add
            i32.store offset=4
            get_local $l11
            get_local $l9
            get_local $l10
            i32.mul
            i32.add
            get_local $l7
            i32.add
            set_local $l0
            i32.const 0
            set_local $l6
            block $B42 (result i32)
              get_local $l18
              i32.const 1
              i32.ne
              if $I43
                get_local $l0
                get_local $l3
                get_local $p0
                i32.load offset=2504
                get_local $l21
                get_local $p0
                i32.load offset=200
                i32.load offset=13128
                get_local $l19
                i32.mul
                i32.add
                i32.const 148
                i32.mul
                i32.add
                get_local $l2
                i32.add
                i32.load8_u offset=142
                i32.const 3
                i32.eq
                i32.const 2
                i32.shl
                i32.add
                i32.load
                get_local $l4
                call $copy_pixel
                get_local $l1
                set_local $l6
              end
              get_local $l0
              get_local $l6
              i32.add
            end
            get_local $l3
            get_local $p0
            i32.load offset=2504
            get_local $p0
            i32.load offset=200
            i32.load offset=13128
            get_local $l19
            i32.mul
            get_local $l14
            i32.add
            i32.const 148
            i32.mul
            i32.add
            get_local $l2
            i32.add
            i32.load8_u offset=142
            i32.const 3
            i32.eq
            i32.const 2
            i32.shl
            i32.add
            i32.load
            get_local $l6
            i32.add
            get_local $l5
            get_local $l4
            i32.shl
            tee_local $l7
            call $memcpy
            drop
            get_local $l22
            i32.const 1
            i32.eq
            br_if $B41
            get_local $l0
            get_local $l6
            get_local $l7
            i32.add
            tee_local $l7
            i32.add
            get_local $l3
            get_local $p0
            i32.load offset=2504
            get_local $l20
            get_local $p0
            i32.load offset=200
            i32.load offset=13128
            get_local $l19
            i32.mul
            i32.add
            i32.const 148
            i32.mul
            i32.add
            get_local $l2
            i32.add
            i32.load8_u offset=142
            i32.const 3
            i32.eq
            i32.const 2
            i32.shl
            i32.add
            i32.load
            get_local $l7
            i32.add
            get_local $l4
            call $copy_pixel
          end
          i32.const 0
          set_local $l7
          block $B44 (result i32)
            i32.const 0
            get_local $l18
            br_if $B44
            drop
            i32.const 1
            get_local $p0
            i32.load offset=2504
            get_local $l21
            get_local $p0
            i32.load offset=200
            i32.load offset=13128
            get_local $l13
            i32.mul
            i32.add
            i32.const 148
            i32.mul
            i32.add
            get_local $l2
            i32.add
            i32.load8_u offset=142
            i32.const 3
            i32.ne
            br_if $B44
            drop
            get_local $l11
            get_local $l1
            i32.sub
            get_local $p0
            get_local $l15
            i32.add
            i32.load offset=184
            get_local $l25
            get_local $l29
            i32.mul
            get_local $l17
            i32.add
            get_local $l4
            i32.shl
            i32.add
            get_local $l4
            get_local $l9
            get_local $l10
            get_local $l1
            call $copy_vert
            i32.const 0
          end
          set_local $l6
          block $B45
            get_local $l22
            br_if $B45
            i32.const 1
            set_local $l7
            get_local $p0
            i32.load offset=2504
            get_local $l20
            get_local $p0
            i32.load offset=200
            i32.load offset=13128
            get_local $l13
            i32.mul
            i32.add
            i32.const 148
            i32.mul
            i32.add
            get_local $l2
            i32.add
            i32.load8_u offset=142
            i32.const 3
            i32.ne
            br_if $B45
            get_local $l11
            get_local $l5
            get_local $l4
            i32.shl
            i32.add
            get_local $p0
            get_local $l15
            i32.add
            i32.load offset=184
            get_local $l25
            get_local $l28
            i32.mul
            get_local $l17
            i32.add
            get_local $l4
            i32.shl
            i32.add
            get_local $l4
            get_local $l9
            get_local $l10
            get_local $l1
            call $copy_vert
            i32.const 0
            set_local $l7
          end
          get_local $l11
          get_local $l6
          get_local $l4
          i32.shl
          tee_local $l1
          i32.sub
          get_local $l12
          get_local $l1
          i32.sub
          get_local $l5
          get_local $l6
          i32.add
          get_local $l7
          i32.add
          get_local $l4
          i32.shl
          get_local $l9
          get_local $l10
          get_local $l8
          call $copy_CTB
          get_local $p0
          get_local $l12
          get_local $l8
          get_local $l16
          get_local $l17
          get_local $l5
          get_local $l9
          get_local $l2
          get_local $l14
          get_local $l13
          call $copy_CTB_to_hv
          get_local $l12
          get_local $l11
          get_local $l8
          get_local $l10
          get_local $l24
          get_local $l3
          i32.const 16
          i32.add
          get_local $l5
          get_local $l9
          get_local $l2
          get_local $l3
          i32.const 14
          i32.add
          get_local $l3
          i32.const 12
          i32.add
          get_local $l3
          i32.const 8
          i32.add
          get_local $l32
          i32.load
          call_indirect (type $t15)
        end
        get_local $p0
        get_local $l12
        get_local $l11
        get_local $l8
        get_local $l10
        get_local $p1
        get_local $p2
        get_local $l5
        get_local $l9
        get_local $l2
        call $restore_tqb_pixels
        get_local $l34
        i32.const 3
        i32.store8 offset=142
      end
      get_local $l27
      get_local $l2
      i32.const 1
      i32.add
      tee_local $l2
      i32.ne
      if $I46
        get_local $p0
        i32.load offset=200
        tee_local $l6
        i32.load offset=13080
        set_local $l16
        br $L33
      end
    end
    get_local $l3
    i32.const 32
    i32.add
    set_global $g0)
  (func $get_qPy (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    get_local $p0
    i32.load offset=4316
    get_local $p0
    i32.load offset=200
    tee_local $p0
    i32.load offset=13140
    get_local $p2
    get_local $p0
    i32.load offset=13064
    tee_local $p0
    i32.shr_s
    i32.mul
    get_local $p1
    get_local $p0
    i32.shr_s
    i32.add
    i32.add
    i32.load8_s)
  (func $get_pcm (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    i32.const 2
    set_local $l0
    block $B0
      get_local $p1
      get_local $p2
      i32.or
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $p1
      get_local $p0
      i32.load offset=200
      tee_local $p1
      i32.load offset=13084
      tee_local $l1
      i32.shr_s
      tee_local $l2
      get_local $p1
      i32.load offset=13156
      tee_local $l3
      i32.ge_s
      br_if $B0
      get_local $p2
      get_local $l1
      i32.shr_s
      tee_local $p2
      get_local $p1
      i32.load offset=13160
      i32.ge_s
      br_if $B0
      get_local $p0
      i32.load offset=4348
      get_local $p2
      get_local $l3
      i32.mul
      get_local $l2
      i32.add
      i32.add
      i32.load8_u
      set_local $l0
    end
    get_local $l0)
  (func $chroma_tc (type $t9) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (result i32)
    block $B0 (result i32)
      i32.const 0
      get_local $p0
      i32.load offset=204
      i32.const 28
      i32.const 32
      get_local $p2
      i32.const 1
      i32.eq
      select
      i32.add
      i32.load
      get_local $p1
      i32.add
      tee_local $p1
      i32.const 0
      i32.lt_s
      br_if $B0
      drop
      get_local $p1
      i32.const 57
      get_local $p1
      i32.const 57
      i32.lt_s
      select
      set_local $p2
      get_local $p0
      i32.load offset=200
      i32.load offset=4
      i32.const 1
      i32.eq
      if $I1
        get_local $p1
        get_local $p2
        i32.const 30
        i32.lt_s
        br_if $B0
        drop
        get_local $p2
        i32.const 6
        i32.sub
        get_local $p2
        i32.const 44
        i32.ge_s
        br_if $B0
        drop
        get_local $p2
        i32.const 2408
        i32.add
        i32.load8_u
        br $B0
      end
      get_local $p1
      i32.const 51
      get_local $p2
      i32.const 51
      i32.lt_s
      select
    end
    get_local $p3
    i32.add
    i32.const 2
    i32.add
    tee_local $p0
    i32.const 53
    get_local $p0
    i32.const 53
    i32.lt_s
    select
    tee_local $p0
    i32.const 0
    get_local $p0
    i32.const 0
    i32.gt_s
    select
    i32.const 2384
    i32.add
    i32.load8_u)
  (func $copy_CTB (type $t6) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32)
    (local $l0 i32)
    get_local $p3
    i32.const 1
    i32.ge_s
    if $I0
      loop $L1
        get_local $p0
        get_local $p1
        get_local $p2
        call $memcpy
        set_local $p0
        get_local $p1
        get_local $p5
        i32.add
        set_local $p1
        get_local $p0
        get_local $p4
        i32.add
        set_local $p0
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p3
        i32.ne
        br_if $L1
      end
    end)
  (func $copy_CTB_to_hv (type $t14) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (param $p8 i32) (param $p9 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32)
    get_local $p0
    i32.load offset=200
    tee_local $l0
    get_local $p7
    i32.const 2
    i32.shl
    tee_local $p7
    i32.add
    tee_local $l1
    i32.const 13180
    i32.add
    i32.load
    set_local $l2
    get_local $l0
    i32.load offset=13124
    set_local $l3
    get_local $p0
    get_local $p7
    i32.add
    tee_local $p7
    tee_local $l4
    i32.const 172
    i32.add
    i32.load
    get_local $l0
    i32.load offset=13120
    get_local $l1
    i32.const 13168
    i32.add
    i32.load
    i32.shr_s
    tee_local $l1
    get_local $p9
    i32.const 1
    i32.shl
    tee_local $p9
    i32.mul
    get_local $p3
    i32.add
    get_local $l0
    i32.load offset=56
    tee_local $p0
    i32.shl
    i32.add
    get_local $p1
    get_local $p5
    get_local $p0
    i32.shl
    tee_local $l0
    call $memcpy
    drop
    get_local $l4
    i32.load offset=172
    get_local $l1
    get_local $p9
    i32.const 1
    i32.or
    i32.mul
    get_local $p3
    i32.add
    get_local $p0
    i32.shl
    i32.add
    get_local $p1
    get_local $p6
    i32.const 1
    i32.sub
    get_local $p2
    i32.mul
    i32.add
    get_local $l0
    call $memcpy
    drop
    get_local $p7
    i32.load offset=184
    get_local $l3
    get_local $l2
    i32.shr_s
    tee_local $p3
    get_local $p8
    i32.const 1
    i32.shl
    tee_local $p8
    i32.mul
    get_local $p4
    i32.add
    get_local $p0
    i32.shl
    i32.add
    get_local $p1
    get_local $p0
    get_local $p6
    i32.const 1
    get_local $p0
    i32.shl
    tee_local $p9
    get_local $p2
    call $copy_vert
    get_local $p7
    i32.load offset=184
    get_local $p3
    get_local $p8
    i32.const 1
    i32.or
    i32.mul
    get_local $p4
    i32.add
    get_local $p0
    i32.shl
    i32.add
    get_local $p1
    get_local $p5
    i32.const 1
    i32.sub
    get_local $p0
    i32.shl
    i32.add
    get_local $p0
    get_local $p6
    get_local $p9
    get_local $p2
    call $copy_vert)
  (func $copy_pixel (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    get_local $p2
    if $I0
      get_local $p0
      get_local $p1
      i32.load16_u
      i32.store16
      return
    end
    get_local $p0
    get_local $p1
    i32.load8_u
    i32.store8)
  (func $copy_vert (type $t6) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32)
    block $B0
      get_local $p2
      if $I1
        i32.const 0
        set_local $p2
        get_local $p3
        i32.const 0
        i32.le_s
        br_if $B0
        loop $L2
          get_local $p0
          get_local $p1
          i32.load16_u
          i32.store16
          get_local $p1
          get_local $p5
          i32.add
          set_local $p1
          get_local $p0
          get_local $p4
          i32.add
          set_local $p0
          get_local $p2
          i32.const 1
          i32.add
          tee_local $p2
          get_local $p3
          i32.ne
          br_if $L2
        end
        br $B0
      end
      get_local $p3
      i32.const 1
      i32.lt_s
      br_if $B0
      i32.const 0
      set_local $p2
      loop $L3
        get_local $p0
        get_local $p1
        i32.load8_u
        i32.store8
        get_local $p1
        get_local $p5
        i32.add
        set_local $p1
        get_local $p0
        get_local $p4
        i32.add
        set_local $p0
        get_local $p2
        i32.const 1
        i32.add
        tee_local $p2
        get_local $p3
        i32.ne
        br_if $L3
      end
    end)
  (func $restore_tqb_pixels (type $t14) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (param $p8 i32) (param $p9 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32)
    get_local $p0
    i32.load offset=200
    set_local $l0
    block $B0
      get_local $p0
      i32.load offset=204
      i32.load8_u offset=40
      i32.eqz
      if $I1
        get_local $l0
        i32.const 13056
        i32.add
        i32.load8_u
        i32.eqz
        br_if $B0
        get_local $l0
        i32.load offset=68
        i32.eqz
        br_if $B0
      end
      get_local $p6
      get_local $l0
      i32.load offset=13084
      tee_local $l1
      i32.shr_s
      tee_local $l3
      get_local $p6
      get_local $p8
      i32.add
      get_local $l1
      i32.shr_s
      tee_local $l5
      i32.ge_s
      br_if $B0
      i32.const 1
      get_local $l1
      i32.shl
      tee_local $p8
      get_local $l0
      get_local $p9
      i32.const 2
      i32.shl
      i32.add
      tee_local $l2
      i32.const 13168
      i32.add
      i32.load
      tee_local $l6
      i32.shr_s
      get_local $l0
      i32.load offset=56
      i32.shl
      set_local $l7
      get_local $p5
      get_local $p7
      i32.add
      get_local $l1
      i32.shr_s
      set_local $l4
      get_local $p5
      get_local $l1
      i32.shr_s
      set_local $p9
      get_local $p8
      get_local $l2
      i32.const 13180
      i32.add
      i32.load
      tee_local $l8
      i32.shr_s
      set_local $l1
      loop $L2
        get_local $p9
        get_local $l4
        i32.lt_s
        if $I3
          get_local $l3
          get_local $p6
          i32.sub
          set_local $l9
          get_local $p9
          set_local $p7
          loop $L4
            get_local $p0
            i32.load offset=4348
            get_local $p0
            i32.load offset=200
            tee_local $p8
            i32.load offset=13156
            get_local $l3
            i32.mul
            get_local $p7
            i32.add
            i32.add
            i32.load8_u
            i32.eqz
            get_local $l1
            i32.const 1
            i32.lt_s
            i32.or
            i32.eqz
            if $I5
              get_local $p1
              get_local $l9
              get_local $p8
              i32.load offset=13084
              tee_local $l0
              i32.shl
              get_local $l8
              i32.shr_s
              tee_local $l10
              get_local $p3
              i32.mul
              i32.add
              get_local $p7
              get_local $p5
              i32.sub
              get_local $l0
              i32.shl
              get_local $l6
              i32.shr_s
              get_local $p8
              i32.load offset=56
              i32.shl
              tee_local $p8
              i32.add
              set_local $l2
              get_local $p2
              get_local $p4
              get_local $l10
              i32.mul
              i32.add
              get_local $p8
              i32.add
              set_local $l0
              i32.const 0
              set_local $p8
              loop $L6
                get_local $l2
                get_local $l0
                get_local $l7
                call $memcpy
                set_local $l2
                get_local $p4
                get_local $l0
                i32.add
                set_local $l0
                get_local $p3
                get_local $l2
                i32.add
                set_local $l2
                get_local $p8
                i32.const 1
                i32.add
                tee_local $p8
                get_local $l1
                i32.ne
                br_if $L6
              end
            end
            get_local $p7
            i32.const 1
            i32.add
            tee_local $p7
            get_local $l4
            i32.ne
            br_if $L4
          end
        end
        get_local $l3
        i32.const 1
        i32.add
        tee_local $l3
        get_local $l5
        i32.ne
        br_if $L2
      end
    end)
  (func $ff_hevc_hls_filters (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load offset=200
    tee_local $l0
    i32.load offset=13124
    set_local $l1
    get_local $l0
    i32.load offset=13120
    set_local $l0
    get_local $p1
    i32.eqz
    get_local $p2
    i32.eqz
    i32.or
    i32.eqz
    if $I0
      get_local $p0
      get_local $p1
      get_local $p3
      i32.sub
      get_local $p2
      get_local $p3
      i32.sub
      get_local $p3
      call $ff_hevc_hls_filter
    end
    get_local $p2
    i32.eqz
    get_local $l0
    get_local $p3
    i32.sub
    get_local $p1
    i32.gt_s
    i32.or
    i32.eqz
    if $I1
      get_local $p0
      get_local $p1
      get_local $p2
      get_local $p3
      i32.sub
      get_local $p3
      call $ff_hevc_hls_filter
    end
    get_local $p1
    i32.eqz
    get_local $l1
    get_local $p3
    i32.sub
    get_local $p2
    i32.gt_s
    i32.or
    i32.eqz
    if $I2
      get_local $p0
      get_local $p1
      get_local $p3
      i32.sub
      get_local $p2
      get_local $p3
      call $ff_hevc_hls_filter
    end)
  (func $ff_hevc_extract_rbsp (type $t9) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32)
    get_local $p0
    i32.const 0
    i32.store offset=4376
    block $B0
      get_local $p2
      i32.const 2
      i32.lt_s
      br_if $B0
      loop $L1
        block $B2
          get_local $p1
          get_local $l1
          tee_local $l0
          i32.add
          i32.load8_u
          if $I3
            get_local $l0
            i32.const 2
            i32.add
            set_local $l1
            br $B2
          end
          get_local $l0
          i32.const 1
          i32.ge_s
          if $I4
            get_local $l0
            get_local $l0
            i32.const 1
            i32.sub
            tee_local $l1
            get_local $p1
            get_local $l1
            i32.add
            i32.load8_u
            select
            set_local $l0
          end
          get_local $l0
          i32.const 2
          i32.add
          tee_local $l1
          get_local $p2
          i32.ge_s
          br_if $B2
          get_local $p1
          get_local $l0
          i32.add
          i32.load8_u offset=1
          br_if $B2
          get_local $p1
          get_local $l1
          i32.add
          i32.load8_u
          tee_local $l2
          i32.const 3
          i32.gt_u
          br_if $B2
          get_local $p2
          get_local $l0
          get_local $l2
          i32.const 3
          i32.eq
          select
          set_local $p2
          br $B0
        end
        get_local $l0
        i32.const 3
        i32.add
        get_local $p2
        i32.lt_s
        br_if $L1
      end
      get_local $l1
      set_local $l0
    end
    get_local $p2
    i32.const 1
    i32.sub
    get_local $l0
    i32.le_s
    if $I5
      get_local $p3
      get_local $p2
      i32.store offset=8
      get_local $p3
      get_local $p1
      i32.store offset=12
      get_local $p2
      return
    end
    get_local $p3
    get_local $p3
    i32.const 4
    i32.add
    get_local $p2
    i32.const 32
    i32.add
    call $ff_fast_malloc
    block $B6
      get_local $p3
      i32.load
      tee_local $l1
      i32.eqz
      br_if $B6
      get_local $l1
      get_local $p1
      get_local $l0
      call $memcpy
      set_local $l4
      block $B7
        block $B8
          block $B9
            get_local $p2
            get_local $l0
            i32.const 2
            i32.add
            tee_local $l2
            i32.le_s
            if $I10
              get_local $l0
              set_local $l1
              br $B9
            end
            get_local $p0
            i32.const 4380
            i32.add
            set_local $l6
            get_local $l0
            set_local $l1
            loop $L11
              block $B12 (result i32)
                block $B13
                  get_local $p1
                  get_local $l2
                  i32.add
                  tee_local $l3
                  i32.load8_u
                  tee_local $l5
                  i32.const 4
                  i32.ge_u
                  if $I14
                    get_local $l0
                    get_local $l4
                    i32.add
                    tee_local $l5
                    get_local $p1
                    get_local $l1
                    i32.add
                    tee_local $l1
                    i32.load8_u
                    i32.store8
                    get_local $l5
                    get_local $l1
                    i32.load8_u offset=1
                    i32.store8 offset=1
                    get_local $l0
                    i32.const 2
                    i32.add
                    set_local $l0
                    get_local $l3
                    i32.load8_u
                    set_local $l3
                    br $B13
                  end
                  block $B15
                    get_local $p1
                    get_local $l1
                    i32.add
                    tee_local $l2
                    i32.load8_u
                    tee_local $l3
                    br_if $B15
                    i32.const 0
                    set_local $l3
                    get_local $l2
                    i32.load8_u offset=1
                    br_if $B15
                    get_local $l5
                    i32.const 3
                    i32.ne
                    br_if $B8
                    get_local $l0
                    get_local $l4
                    i32.add
                    i32.const 0
                    i32.store16 align=1
                    get_local $p0
                    get_local $p0
                    i32.load offset=4376
                    tee_local $l2
                    i32.const 1
                    i32.add
                    i32.store offset=4376
                    get_local $l1
                    i32.const 3
                    i32.add
                    set_local $l1
                    get_local $l0
                    i32.const 2
                    i32.add
                    set_local $l3
                    block $B16
                      block $B17
                        get_local $l2
                        get_local $p0
                        i32.load offset=4384
                        tee_local $l5
                        i32.ge_s
                        if $I18
                          get_local $p0
                          get_local $l5
                          i32.const 1
                          i32.shl
                          tee_local $l2
                          i32.store offset=4384
                          get_local $l6
                          get_local $l2
                          call $av_reallocp_array
                          get_local $p0
                          i32.load offset=4380
                          tee_local $l2
                          i32.eqz
                          br_if $B6
                          br $B17
                        end
                        get_local $l6
                        i32.load
                        tee_local $l2
                        i32.eqz
                        br_if $B16
                      end
                      get_local $p0
                      i32.load offset=4376
                      i32.const 2
                      i32.shl
                      get_local $l2
                      i32.add
                      i32.const 4
                      i32.sub
                      get_local $l0
                      i32.const 1
                      i32.add
                      i32.store
                    end
                    get_local $l3
                    br $B12
                  end
                  get_local $l1
                  set_local $l2
                end
                get_local $l0
                get_local $l4
                i32.add
                get_local $l3
                i32.store8
                get_local $l2
                i32.const 1
                i32.add
                set_local $l1
                get_local $l0
                i32.const 1
                i32.add
              end
              set_local $l0
              get_local $l1
              i32.const 2
              i32.add
              tee_local $l2
              get_local $p2
              i32.lt_s
              br_if $L11
            end
          end
          get_local $p2
          get_local $l1
          i32.le_s
          br_if $B8
          loop $L19
            get_local $l0
            get_local $l4
            i32.add
            get_local $p1
            get_local $l1
            i32.add
            i32.load8_u
            i32.store8
            get_local $l0
            i32.const 1
            i32.add
            set_local $l0
            get_local $l1
            i32.const 1
            i32.add
            tee_local $l1
            get_local $p2
            i32.ne
            br_if $L19
          end
          br $B7
        end
        get_local $l1
        set_local $p2
      end
      get_local $l0
      get_local $l4
      i32.add
      tee_local $p0
      i64.const 0
      i64.store align=1
      get_local $p0
      i64.const 0
      i64.store offset=24 align=1
      get_local $p0
      i64.const 0
      i64.store offset=16 align=1
      get_local $p0
      i64.const 0
      i64.store offset=8 align=1
      get_local $p3
      get_local $l0
      i32.store offset=8
      get_local $p3
      get_local $l4
      i32.store offset=12
      get_local $p2
      return
    end
    i32.const -48)
  (func $hevc_decode_init (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load offset=60
    set_local $l0
    i32.const 4768
    i32.load8_u
    i32.eqz
    if $I0
      call $cabac_tableinit
      i32.const 4768
      i32.const 1
      i32.store8
    end
    call $hevc_transform_init
    get_local $p0
    call $hevc_init_context
    tee_local $l1
    i32.const 0
    i32.ge_s
    if $I1 (result i32)
      get_local $l0
      i32.const 0
      i32.store offset=4520
      get_local $l0
      i32.const 0
      i32.store offset=4368
      get_local $l0
      get_local $p0
      i32.load offset=808
      tee_local $l1
      i32.const 2
      i32.and
      if $I2 (result i32)
        get_local $p0
        i32.load offset=800
      else
        i32.const 1
      end
      i32.store8 offset=141
      get_local $l0
      block $B3 (result i32)
        get_local $l1
        i32.const 1
        i32.and
        if $I4
          i32.const 1
          get_local $p0
          i32.load offset=800
          i32.const 1
          i32.gt_s
          br_if $B3
          drop
        end
        i32.const 2
      end
      i32.store8 offset=140
      i32.const 0
    else
      get_local $l1
    end)
  (func $hevc_init_context (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load offset=60
    tee_local $l0
    get_local $p0
    i32.store offset=4
    get_local $l0
    i32.const 31328
    call $av_mallocz
    tee_local $l1
    i32.store offset=136
    block $B0
      get_local $l1
      i32.eqz
      br_if $B0
      get_local $l0
      get_local $l1
      i32.store offset=72
      get_local $l0
      get_local $l0
      i32.store offset=8
      get_local $l0
      i32.const 199
      call $av_malloc
      tee_local $l1
      i32.store offset=152
      get_local $l1
      i32.eqz
      br_if $B0
      get_local $l0
      call $av_frame_alloc
      tee_local $l1
      i32.store offset=164
      get_local $l1
      i32.eqz
      br_if $B0
      get_local $l0
      call $av_frame_alloc
      tee_local $l1
      i32.store offset=2524
      get_local $l1
      i32.eqz
      br_if $B0
      get_local $l0
      i32.const 1
      i32.store8 offset=4469
      get_local $l0
      i32.const 2147483647
      i32.store offset=2592
      get_local $l0
      i32.const 0
      i32.store offset=2584
      get_local $l0
      i32.const 2528
      i32.add
      get_local $l1
      i32.store
      i32.const 0
      return
    end
    get_local $p0
    call $hevc_decode_free
    drop
    i32.const -48)
  (func $hevc_decode_frame (type $t9) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    i32.load offset=60
    set_local $p0
    get_local $p3
    i32.load offset=28
    tee_local $l0
    i32.eqz
    if $I0
      get_local $p0
      get_local $p1
      i32.const 1
      call $ff_hevc_output_frame
      tee_local $p0
      i32.const 0
      i32.lt_s
      if $I1
        get_local $p0
        return
      end
      get_local $p2
      get_local $p0
      i32.store
      i32.const 0
      return
    end
    get_local $p0
    i32.const 1
    i32.store16 offset=4524
    get_local $p0
    i32.const 0
    i32.store offset=2520
    get_local $p0
    get_local $p3
    i32.load offset=24
    get_local $l0
    call $decode_nal_units
    tee_local $l0
    i32.const 0
    i32.ge_s
    if $I2 (result i32)
      get_local $p0
      i32.load offset=2604
      if $I3
        get_local $p0
        i32.const 0
        i32.store offset=2604
      end
      get_local $p0
      i32.load offset=164
      tee_local $l0
      i32.load offset=304
      if $I4
        get_local $l0
        get_local $p0
        i64.load16_u offset=4524
        i64.store offset=128
        get_local $p1
        get_local $l0
        i32.const 400
        call $memcpy
        drop
        get_local $l0
        i32.const 0
        i32.const 400
        call $memset
        call $get_frame_defaults
        get_local $p2
        i32.const 1
        i32.store
      end
      get_local $p3
      i32.load offset=28
    else
      get_local $l0
    end)
  (func $decode_nal_units (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32)
    get_local $p0
    i32.const 0
    i32.store offset=2520
    get_local $p0
    i32.const 0
    i32.store offset=4408
    get_local $p0
    i32.load offset=2584
    set_local $l0
    get_local $p0
    i32.const 0
    i32.store offset=2584
    get_local $p0
    get_local $l0
    i32.store offset=2588
    block $B0
      get_local $p2
      i32.const 4
      i32.lt_s
      br_if $B0
      get_local $p0
      i32.const 4392
      i32.add
      set_local $l5
      get_local $p0
      i32.const 4396
      i32.add
      set_local $l6
      get_local $p0
      i32.const 4388
      i32.add
      set_local $l7
      block $B1
        loop $L2
          block $B3 (result i32)
            block $B4
              block $B5
                get_local $p0
                i32.load8_u offset=4470
                if $I6
                  i32.const 0
                  set_local $l1
                  i32.const 0
                  set_local $l2
                  get_local $p0
                  i32.load offset=4480
                  tee_local $l0
                  i32.const 1
                  i32.ge_s
                  if $I7
                    loop $L8
                      get_local $p1
                      get_local $l1
                      i32.add
                      i32.load8_u
                      get_local $l2
                      i32.const 8
                      i32.shl
                      i32.or
                      set_local $l2
                      get_local $l1
                      i32.const 1
                      i32.add
                      tee_local $l1
                      get_local $l0
                      i32.ne
                      br_if $L8
                    end
                  end
                  get_local $l2
                  get_local $p2
                  get_local $l0
                  i32.sub
                  tee_local $l3
                  i32.gt_s
                  br_if $B5
                  get_local $p1
                  get_local $l0
                  i32.add
                  br $B3
                end
                loop $L9
                  get_local $p2
                  set_local $l0
                  block $B10
                    get_local $p1
                    i32.load8_u
                    br_if $B10
                    get_local $p1
                    i32.load8_u offset=1
                    br_if $B10
                    get_local $p1
                    i32.load8_u offset=2
                    i32.const 1
                    i32.eq
                    br_if $B4
                  end
                  get_local $l0
                  i32.const 1
                  i32.sub
                  set_local $p2
                  get_local $p1
                  i32.const 1
                  i32.add
                  set_local $p1
                  get_local $l0
                  i32.const 5
                  i32.ge_s
                  br_if $L9
                end
              end
              i32.const -1094995529
              set_local $l1
              br $B0
            end
            get_local $l0
            i32.const 3
            i32.sub
            tee_local $l2
            set_local $l3
            get_local $p1
            i32.const 3
            i32.add
          end
          set_local $l4
          get_local $p0
          i32.load offset=4412
          tee_local $l0
          get_local $p0
          i32.load offset=4408
          tee_local $p1
          i32.le_s
          if $I11
            block $B12 (result i32)
              get_local $p0
              i32.load offset=4404
              set_local $p2
              get_local $l0
              i32.const 1
              i32.add
              tee_local $l0
              tee_local $p1
              i32.const 134217727
              i32.lt_u
              if $I13 (result i32)
                get_local $p2
                get_local $p1
                i32.const 4
                i32.shl
                call $av_realloc
              else
                i32.const 0
              end
              tee_local $p2
              i32.eqz
            end
            if $I14
              i32.const -48
              set_local $l1
              br $B0
            end
            get_local $p0
            get_local $p2
            i32.store offset=4404
            get_local $p2
            get_local $p0
            i32.load offset=4412
            tee_local $p1
            i32.const 4
            i32.shl
            i32.add
            i32.const 0
            get_local $l0
            get_local $p1
            i32.sub
            i32.const 4
            i32.shl
            call $memset
            drop
            get_local $l7
            get_local $l0
            call $av_reallocp_array
            get_local $l6
            get_local $l0
            call $av_reallocp_array
            get_local $l5
            get_local $l0
            call $av_reallocp_array
            get_local $p0
            i32.load offset=4396
            tee_local $p1
            get_local $p0
            i32.load offset=4412
            i32.const 2
            i32.shl
            i32.add
            i32.const 1024
            i32.store
            get_local $p1
            get_local $p0
            i32.load offset=4412
            i32.const 2
            i32.shl
            i32.add
            i32.load
            i32.const 4
            call $av_malloc_array
            set_local $p1
            get_local $p0
            i32.load offset=4392
            get_local $p0
            i32.load offset=4412
            i32.const 2
            i32.shl
            i32.add
            get_local $p1
            i32.store
            get_local $p0
            get_local $l0
            i32.store offset=4412
            get_local $p0
            i32.load offset=4408
            set_local $p1
          end
          get_local $p0
          get_local $p1
          i32.const 2
          i32.shl
          tee_local $p2
          get_local $p0
          i32.load offset=4396
          i32.add
          i32.load
          i32.store offset=4384
          get_local $p0
          get_local $p0
          i32.load offset=4392
          get_local $p2
          i32.add
          i32.load
          i32.store offset=4380
          get_local $p0
          get_local $l4
          get_local $l2
          get_local $p0
          i32.load offset=4404
          get_local $p1
          i32.const 4
          i32.shl
          i32.add
          tee_local $p2
          call $ff_hevc_extract_rbsp
          set_local $l1
          get_local $p0
          i32.load offset=4388
          get_local $p0
          i32.load offset=4408
          i32.const 2
          i32.shl
          i32.add
          get_local $p0
          i32.load offset=4376
          i32.store
          get_local $p0
          i32.load offset=4396
          get_local $p0
          i32.load offset=4408
          i32.const 2
          i32.shl
          i32.add
          get_local $p0
          i32.load offset=4384
          i32.store
          get_local $p0
          get_local $p0
          i32.load offset=4408
          tee_local $p1
          i32.const 1
          i32.add
          i32.store offset=4408
          get_local $p0
          i32.load offset=4392
          get_local $p1
          i32.const 2
          i32.shl
          i32.add
          get_local $p0
          i32.load offset=4380
          i32.store
          get_local $l1
          i32.const 0
          i32.lt_s
          br_if $B0
          get_local $p0
          i32.load offset=136
          i32.const 204
          i32.add
          get_local $p2
          i32.load offset=12
          get_local $p2
          i32.load offset=8
          call $init_get_bits8
          tee_local $l0
          i32.const 0
          i32.lt_s
          br_if $B1
          get_local $p0
          call $hls_nal_unit
          drop
          get_local $p0
          i32.load offset=2512
          i32.const -2
          i32.and
          i32.const 36
          i32.eq
          if $I15
            get_local $p0
            i32.const 1
            i32.store offset=2584
          end
          get_local $l1
          get_local $l4
          i32.add
          set_local $p1
          get_local $l3
          get_local $l1
          i32.sub
          tee_local $p2
          i32.const 3
          i32.gt_s
          br_if $L2
        end
        get_local $p0
        i32.load offset=4408
        i32.const 1
        i32.lt_s
        br_if $B1
        i32.const 0
        set_local $p1
        loop $L16
          get_local $p0
          get_local $p1
          i32.const 2
          i32.shl
          tee_local $p2
          get_local $p0
          i32.load offset=4388
          i32.add
          i32.load
          i32.store offset=4376
          get_local $p0
          get_local $p0
          i32.load offset=4392
          get_local $p2
          i32.add
          i32.load
          i32.store offset=4380
          get_local $p0
          get_local $p0
          i32.load offset=4404
          get_local $p1
          i32.const 4
          i32.shl
          i32.add
          tee_local $p2
          i32.load offset=12
          get_local $p2
          i32.load offset=8
          call $decode_nal_unit
          i32.const -1
          i32.le_s
          br_if $B1
          get_local $p1
          i32.const 1
          i32.add
          tee_local $p1
          get_local $p0
          i32.load offset=4408
          i32.lt_s
          br_if $L16
        end
      end
      get_local $l0
      set_local $l1
    end
    block $B17
      get_local $p0
      i32.load offset=2520
      i32.eqz
      br_if $B17
      get_local $p0
      i32.load8_u offset=140
      i32.const 1
      i32.ne
      br_if $B17
    end
    get_local $l1)
  (func $hevc_decode_free (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p0
    i32.load offset=60
    tee_local $p0
    call $pic_arrays_free
    get_local $p0
    i32.load offset=4412
    i32.const 1
    i32.ge_s
    if $I0
      loop $L1
        get_local $p0
        i32.load offset=4392
        get_local $l0
        i32.const 2
        i32.shl
        i32.add
        call $av_freep
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p0
        i32.load offset=4412
        i32.lt_s
        br_if $L1
      end
    end
    get_local $p0
    i32.const 4396
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4388
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4392
    i32.add
    call $av_freep
    get_local $p0
    i32.const 152
    i32.add
    call $av_freep
    get_local $p0
    i32.const 168
    i32.add
    call $av_freep
    i32.const 0
    set_local $l0
    loop $L2
      get_local $p0
      get_local $l0
      i32.const 2
      i32.shl
      i32.add
      tee_local $l1
      i32.const 172
      i32.add
      call $av_freep
      get_local $l1
      i32.const 184
      i32.add
      call $av_freep
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 3
      i32.ne
      br_if $L2
    end
    get_local $p0
    i32.const 164
    i32.add
    call $av_frame_free
    get_local $p0
    get_local $p0
    i32.const 2524
    i32.add
    tee_local $l0
    i32.const -1
    call $ff_hevc_unref_frame
    get_local $l0
    call $av_frame_free
    i32.const 0
    set_local $l0
    i32.const 0
    set_local $l1
    loop $L3
      get_local $p0
      get_local $l1
      i32.const 2
      i32.shl
      i32.add
      i32.const 208
      i32.add
      call $av_buffer_unref
      get_local $l1
      i32.const 1
      i32.add
      tee_local $l1
      i32.const 16
      i32.ne
      br_if $L3
    end
    loop $L4
      get_local $p0
      get_local $l0
      i32.const 2
      i32.shl
      i32.add
      i32.const 272
      i32.add
      call $av_buffer_unref
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 32
      i32.ne
      br_if $L4
    end
    i32.const 0
    set_local $l0
    loop $L5
      get_local $p0
      get_local $l0
      i32.const 2
      i32.shl
      i32.add
      i32.const 400
      i32.add
      call $av_buffer_unref
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 256
      i32.ne
      br_if $L5
    end
    get_local $p0
    i32.const 0
    i32.store offset=204
    get_local $p0
    i64.const 0
    i64.store offset=196 align=4
    get_local $p0
    i32.const 1424
    i32.add
    call $av_buffer_unref
    get_local $p0
    i32.const 2096
    i32.add
    call $av_freep
    get_local $p0
    i32.const 2100
    i32.add
    call $av_freep
    get_local $p0
    i32.const 2104
    i32.add
    call $av_freep
    get_local $p0
    i32.load8_u offset=141
    tee_local $l1
    i32.const 2
    i32.ge_u
    if $I6
      i32.const 1
      set_local $l0
      loop $L7
        get_local $p0
        get_local $l0
        i32.const 2
        i32.shl
        i32.add
        tee_local $l2
        i32.const 72
        i32.add
        tee_local $l3
        i32.load
        if $I8
          get_local $l3
          call $av_freep
          get_local $l2
          i32.const 8
          i32.add
          call $av_freep
          get_local $p0
          i32.load8_u offset=141
          set_local $l1
        end
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $l1
        i32.const 255
        i32.and
        i32.lt_u
        br_if $L7
      end
    end
    get_local $p0
    i32.load offset=136
    get_local $p0
    i32.load offset=72
    i32.eq
    if $I9
      get_local $p0
      i32.const 0
      i32.store offset=136
    end
    get_local $p0
    i32.const 72
    i32.add
    call $av_freep
    get_local $p0
    i32.load offset=4412
    i32.const 1
    i32.ge_s
    if $I10
      i32.const 0
      set_local $l0
      loop $L11
        get_local $p0
        i32.load offset=4404
        get_local $l0
        i32.const 4
        i32.shl
        i32.add
        call $av_freep
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p0
        i32.load offset=4412
        i32.lt_s
        br_if $L11
      end
    end
    get_local $p0
    i32.const 4404
    i32.add
    call $av_freep
    get_local $p0
    i32.const 0
    i32.store offset=4412
    i32.const 0)
  (func $pic_arrays_free (type $t1) (param $p0 i32)
    get_local $p0
    i32.const 2504
    i32.add
    call $av_freep
    get_local $p0
    i32.const 2508
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4332
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4336
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4340
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4344
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4348
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4316
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4328
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4352
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4320
    i32.add
    call $av_freep
    get_local $p0
    i32.const 4324
    i32.add
    call $av_freep
    get_local $p0
    i32.const 2096
    i32.add
    call $av_freep
    get_local $p0
    i32.const 2104
    i32.add
    call $av_freep
    get_local $p0
    i32.const 2100
    i32.add
    call $av_freep)
  (func $hevc_decode_flush (type $t1) (param $p0 i32)
    (local $l0 i32)
    get_local $p0
    i32.load offset=60
    tee_local $p0
    tee_local $l0
    get_local $l0
    i32.const 2524
    i32.add
    i32.const -1
    call $ff_hevc_unref_frame
    get_local $p0
    i32.const 2147483647
    i32.store offset=2592)
  (func $init_get_bits8 (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    get_local $p0
    get_local $p1
    i32.const -8
    get_local $p2
    i32.const 3
    i32.shl
    get_local $p2
    i32.const 268435455
    i32.gt_u
    select
    call $init_get_bits)
  (func $hls_nal_unit (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    i32.const -1094995529
    set_local $l1
    block $B0
      get_local $p0
      i32.load offset=136
      i32.const 204
      i32.add
      tee_local $l0
      call $get_bits1
      br_if $B0
      get_local $p0
      get_local $l0
      i32.const 6
      call $get_bits
      i32.store offset=2512
      get_local $l0
      i32.const 6
      call $get_bits
      set_local $l2
      get_local $p0
      get_local $l0
      i32.const 3
      call $get_bits
      i32.const 1
      i32.sub
      tee_local $p0
      i32.store offset=2516
      get_local $p0
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $l2
      i32.eqz
      set_local $l1
    end
    get_local $l1)
  (func $decode_nal_unit (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    block $B0
      get_local $p0
      i32.load offset=136
      i32.const 204
      i32.add
      get_local $p1
      get_local $p2
      call $init_get_bits8
      tee_local $p1
      i32.const 0
      i32.lt_s
      br_if $B0
      block $B1
        get_local $p0
        call $hls_nal_unit
        tee_local $p2
        i32.const 0
        i32.lt_s
        br_if $B1
        i32.const 0
        set_local $p1
        get_local $p2
        i32.eqz
        br_if $B0
        block $B2
          get_local $p0
          block $B3 (result i32)
            block $B4
              block $B5
                block $B6
                  block $B7
                    block $B8
                      block $B9
                        get_local $p0
                        i32.load offset=2512
                        br_table $B8 $B8 $B8 $B8 $B8 $B8 $B8 $B8 $B8 $B8 $B0 $B0 $B0 $B0 $B0 $B0 $B8 $B8 $B8 $B8 $B8 $B8 $B0 $B0 $B0 $B0 $B0 $B0 $B0 $B0 $B0 $B0 $B0 $B0 $B6 $B0 $B7 $B7 $B0 $B9 $B9 $B0 $B0 $B0 $B0 $B0 $B0 $B0 $B5 $B0
                      end
                      get_local $p0
                      set_local $p2
                      loop $L10
                        get_local $p2
                        call $decode_nal_sei_message
                        block $B11 (result i32)
                          i32.const 0
                          get_local $p2
                          i32.load offset=136
                          i32.const 204
                          i32.add
                          tee_local $l0
                          call $get_bits_left
                          i32.const 1
                          i32.lt_s
                          br_if $B11
                          drop
                          get_local $l0
                          call $show_bits
                          i32.const 128
                          i32.ne
                        end
                        br_if $L10
                      end
                      i32.const 1
                      tee_local $p2
                      i32.const 0
                      i32.lt_u
                      br_if $B1
                      br $B0
                    end
                    get_local $p0
                    call $hls_slice_header
                    tee_local $p2
                    i32.const 0
                    i32.lt_s
                    if $I12
                      get_local $p2
                      return
                    end
                    get_local $p0
                    i32.load offset=2512
                    set_local $l0
                    get_local $p0
                    i32.load offset=2592
                    tee_local $p1
                    i32.const 2147483647
                    i32.ne
                    br_if $B2
                    get_local $l0
                    i32.const 21
                    i32.gt_u
                    if $I13
                      i32.const 2147483647
                      set_local $p1
                      br $B2
                    end
                    i32.const 1
                    get_local $l0
                    i32.shl
                    tee_local $l1
                    i32.const 2555904
                    i32.and
                    br_if $B4
                    i32.const 2147483647
                    set_local $p1
                    get_local $l1
                    i32.const 1572864
                    i32.and
                    i32.eqz
                    br_if $B2
                    i32.const -2147483648
                    br $B3
                  end
                  get_local $p0
                  i32.const 2147483647
                  i32.store offset=2592
                  get_local $p0
                  get_local $p0
                  i32.load16_u offset=4364
                  i32.const 1
                  i32.add
                  i32.const 255
                  i32.and
                  i32.store16 offset=4364
                  i32.const 0
                  return
                end
                get_local $p0
                call $ff_hevc_decode_nal_pps
                tee_local $p2
                i32.const 0
                i32.ge_s
                br_if $B0
                br $B1
              end
              get_local $p0
              call $ff_hevc_decode_nal_sps
              tee_local $p2
              i32.const 0
              i32.lt_s
              br_if $B1
              br $B0
            end
            get_local $p0
            i32.load offset=2572
          end
          tee_local $p1
          i32.store offset=2592
        end
        block $B14
          get_local $l0
          i32.const -2
          i32.and
          i32.const 8
          i32.ne
          br_if $B14
          get_local $p1
          get_local $p0
          i32.load offset=2572
          i32.ge_s
          if $I15
            get_local $p0
            i32.const 0
            i32.store offset=2604
            i32.const 0
            return
          end
          get_local $l0
          i32.const 9
          i32.ne
          br_if $B14
          get_local $p0
          i32.const -2147483648
          i32.store offset=2592
        end
        block $B16
          get_local $p0
          i32.const 1448
          i32.add
          i32.load8_u
          if $I17
            get_local $p0
            call $hevc_frame_start
            tee_local $p1
            i32.const 0
            i32.lt_s
            br_if $B0
            get_local $p0
            i32.load offset=2512
            set_local $l0
            br $B16
          end
          get_local $p0
          i32.load offset=2520
          i32.eqz
          br_if $B1
        end
        i32.const -1094995529
        set_local $p1
        get_local $l0
        get_local $p0
        i32.load offset=4416
        i32.ne
        br_if $B0
        get_local $p0
        call $hls_slice_data
        tee_local $p2
        get_local $p0
        i32.load offset=200
        tee_local $p1
        i32.load offset=13132
        get_local $p1
        i32.load offset=13128
        i32.mul
        i32.ge_s
        if $I18
          get_local $p0
          i32.const 1
          i32.store offset=2604
        end
        i32.const 0
        set_local $p1
        get_local $p2
        i32.const 0
        i32.ge_s
        br_if $B0
      end
      get_local $p0
      i32.load offset=4
      i32.load offset=688
      i32.const 28
      i32.shl
      i32.const 31
      i32.shr_s
      get_local $p2
      i32.and
      return
    end
    get_local $p1)
  (func $init_get_bits (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    i32.const 0
    i32.store offset=8
    get_local $p0
    get_local $p2
    i32.const 0
    get_local $p1
    i32.const 0
    i32.ne
    get_local $p2
    i32.const 2147483640
    i32.lt_u
    i32.and
    tee_local $p2
    select
    tee_local $l0
    i32.store offset=12
    get_local $p0
    get_local $p1
    i32.const 0
    get_local $p2
    select
    tee_local $p1
    i32.store
    get_local $p0
    get_local $l0
    i32.const 8
    i32.add
    i32.store offset=16
    get_local $p0
    get_local $p1
    get_local $l0
    i32.const 7
    i32.add
    i32.const 3
    i32.shr_s
    i32.add
    i32.store offset=4
    i32.const 0
    i32.const -1094995529
    get_local $p2
    select)
  (func $hls_slice_header (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32)
    get_local $p0
    i32.const 1448
    i32.add
    get_local $p0
    i32.load offset=136
    i32.const 204
    i32.add
    tee_local $l2
    call $get_bits1
    tee_local $l0
    i32.store8
    block $B0
      get_local $l0
      i32.const 255
      i32.and
      i32.eqz
      get_local $p0
      i32.load offset=2512
      tee_local $l0
      i32.const 16
      i32.sub
      i32.const 4
      i32.gt_u
      i32.or
      br_if $B0
      get_local $p0
      i32.const 2147483647
      i32.store offset=2592
      get_local $p0
      get_local $p0
      i32.load16_u offset=4364
      i32.const 1
      i32.add
      i32.const 255
      i32.and
      i32.store16 offset=4364
      get_local $l0
      i32.const 19
      i32.sub
      i32.const 1
      i32.gt_u
      br_if $B0
      get_local $p0
      call $ff_hevc_clear_refs
      get_local $p0
      i32.load offset=2512
      set_local $l0
    end
    get_local $p0
    i32.const 2046
    i32.add
    i32.const 0
    i32.store8
    get_local $l0
    i32.const -8
    i32.and
    i32.const 16
    i32.eq
    if $I1
      get_local $p0
      get_local $l2
      call $get_bits1
      i32.store8 offset=2046
    end
    get_local $p0
    get_local $l2
    call $get_ue_golomb_long
    tee_local $l0
    i32.store offset=1428
    block $B2
      get_local $l0
      i32.const 255
      i32.gt_u
      br_if $B2
      get_local $p0
      get_local $l0
      i32.const 2
      i32.shl
      i32.add
      i32.load offset=400
      tee_local $l0
      i32.eqz
      br_if $B2
      block $B3
        get_local $p0
        i32.load8_u offset=1448
        tee_local $l3
        if $I4
          get_local $l0
          i32.load offset=4
          set_local $l1
          br $B3
        end
        get_local $p0
        i32.load offset=204
        tee_local $l1
        get_local $l0
        i32.load offset=4
        i32.ne
        br_if $B2
      end
      get_local $p0
      get_local $l1
      i32.store offset=204
      block $B5
        get_local $p0
        i32.load offset=2512
        tee_local $l4
        i32.const 21
        i32.ne
        br_if $B5
        get_local $p0
        i32.load offset=2588
        i32.const 1
        i32.ne
        br_if $B5
        get_local $p0
        i32.const 1
        i32.store8 offset=2046
      end
      block $B6
        block $B7
          get_local $p0
          i32.load offset=200
          tee_local $l0
          get_local $p0
          get_local $l1
          i32.load
          i32.const 2
          i32.shl
          i32.add
          i32.load offset=272
          i32.load offset=4
          tee_local $l1
          i32.ne
          if $I8
            get_local $p0
            get_local $l1
            i32.store offset=200
            block $B9
              get_local $l0
              i32.eqz
              get_local $l4
              i32.const 21
              i32.eq
              i32.or
              get_local $l4
              i32.const -8
              i32.and
              i32.const 16
              i32.ne
              i32.or
              br_if $B9
              block $B10
                get_local $l1
                i32.load offset=13120
                get_local $l0
                i32.load offset=13120
                i32.ne
                br_if $B10
                get_local $l1
                i32.load offset=13124
                get_local $l0
                i32.load offset=13124
                i32.ne
                br_if $B10
                get_local $l1
                i32.load offset=72
                i32.const 12
                i32.mul
                get_local $l1
                i32.add
                i32.const -64
                i32.sub
                i32.load
                get_local $l0
                i32.load offset=72
                i32.const 12
                i32.mul
                get_local $l0
                i32.add
                i32.const -64
                i32.sub
                i32.load
                i32.eq
                br_if $B9
              end
              get_local $p0
              i32.const 0
              i32.store8 offset=2046
            end
            get_local $p0
            call $ff_hevc_clear_refs
            get_local $p0
            get_local $p0
            i32.load offset=200
            call $set_sps
            tee_local $l0
            i32.const 0
            i32.lt_s
            br_if $B7
            get_local $p0
            i32.const 2147483647
            i32.store offset=2592
            get_local $p0
            get_local $p0
            i32.load16_u offset=4364
            i32.const 1
            i32.add
            i32.const 255
            i32.and
            i32.store16 offset=4364
            get_local $p0
            i32.load8_u offset=1448
            set_local $l3
            get_local $p0
            i32.load offset=200
            set_local $l0
          end
          get_local $p0
          i32.load offset=4
          tee_local $l1
          get_local $l0
          i32.load8_u offset=302
          i32.store offset=832
          get_local $l1
          get_local $l0
          i32.load8_u offset=335
          i32.store offset=836
          get_local $p0
          i32.const 1449
          i32.add
          i32.const 0
          i32.store8
          block $B11
            block $B12
              block $B13
                get_local $l3
                i32.eqz
                if $I14
                  get_local $p0
                  i32.const 1432
                  i32.add
                  get_local $l2
                  block $B15 (result i32)
                    get_local $p0
                    i32.load offset=204
                    i32.load8_u offset=41
                    if $I16
                      get_local $p0
                      get_local $l2
                      call $get_bits1
                      i32.store8 offset=1449
                      get_local $p0
                      i32.load offset=200
                      set_local $l0
                    end
                    get_local $l0
                    i32.load offset=13128
                    get_local $l0
                    i32.load offset=13132
                    i32.mul
                    i32.const 1
                    i32.shl
                    i32.const 2
                    i32.sub
                    tee_local $l0
                    i32.const 65535
                    i32.gt_u
                    i32.const 4
                    i32.shl
                    tee_local $l1
                    i32.const 8
                    i32.or
                  end
                  get_local $l1
                  get_local $l0
                  get_local $l0
                  i32.const 16
                  i32.shr_u
                  get_local $l0
                  i32.const 65536
                  i32.lt_u
                  select
                  tee_local $l0
                  i32.const 65280
                  i32.and
                  tee_local $l1
                  select
                  get_local $l0
                  i32.const 8
                  i32.shr_u
                  get_local $l0
                  get_local $l1
                  select
                  i32.const 3264
                  i32.add
                  i32.load8_u
                  i32.add
                  call $get_bits
                  tee_local $l0
                  i32.store
                  get_local $l0
                  get_local $p0
                  i32.load offset=200
                  tee_local $l1
                  i32.load offset=13132
                  get_local $l1
                  i32.load offset=13128
                  i32.mul
                  i32.ge_u
                  br_if $B2
                  get_local $p0
                  i32.load8_u offset=1449
                  br_if $B12
                  get_local $p0
                  i32.const 1436
                  i32.add
                  get_local $l0
                  i32.store
                  get_local $p0
                  get_local $p0
                  i32.load offset=2580
                  i32.const 1
                  i32.add
                  i32.store offset=2580
                  br $B13
                end
                get_local $p0
                i32.const 0
                i32.store offset=2580
                get_local $p0
                i32.const 1432
                i32.add
                i64.const 0
                i64.store align=4
              end
              i32.const 0
              set_local $l0
              get_local $p0
              i32.const 0
              i32.store8 offset=156
              get_local $p0
              i32.load offset=204
              i32.load offset=1624
              i32.const 1
              i32.ge_s
              if $I17
                loop $L18
                  get_local $l2
                  i32.const 1
                  call $skip_bits
                  get_local $l0
                  i32.const 1
                  i32.add
                  tee_local $l0
                  get_local $p0
                  i32.load offset=204
                  i32.load offset=1624
                  i32.lt_s
                  br_if $L18
                end
              end
              get_local $p0
              i32.const 1440
              i32.add
              get_local $l2
              call $get_ue_golomb_long
              tee_local $l0
              i32.store
              get_local $l0
              i32.const 2
              i32.gt_u
              br_if $B2
              get_local $l0
              i32.const 2
              i32.ne
              if $I19
                get_local $p0
                i32.load offset=2512
                i32.const -8
                i32.and
                i32.const 16
                i32.eq
                br_if $B2
              end
              get_local $p0
              i32.const 1450
              i32.add
              i32.const 1
              i32.store8
              get_local $p0
              i32.load offset=204
              i32.load8_u offset=39
              if $I20
                get_local $p0
                get_local $l2
                call $get_bits1
                i32.store8 offset=1450
              end
              get_local $p0
              i32.load offset=200
              i32.load8_u offset=8
              if $I21
                get_local $p0
                i32.const 1451
                i32.add
                get_local $l2
                i32.const 2
                call $get_bits
                i32.store8
              end
              get_local $p0
              i32.load offset=2512
              i32.const 19
              i32.sub
              i32.const 2
              i32.ge_u
              br_if $B6
              get_local $p0
              i32.const 0
              i32.store offset=2572
              get_local $p0
              i32.const 1620
              i32.add
              i32.const 0
              i32.store
              get_local $p0
              i32.load offset=2516
              i32.eqz
              if $I22
                get_local $p0
                i32.const 0
                i32.store offset=2576
              end
              block $B23
                get_local $p0
                i32.load offset=200
                i32.load8_u offset=12941
                if $I24
                  get_local $p0
                  i32.const 2056
                  i32.add
                  get_local $l2
                  call $get_bits1
                  i32.store8
                  get_local $p0
                  i32.load offset=200
                  i32.load offset=4
                  if $I25
                    get_local $p0
                    i32.const 2057
                    i32.add
                    get_local $l2
                    call $get_bits1
                    tee_local $l0
                    i32.store8
                    get_local $p0
                    i32.const 2058
                    i32.add
                    get_local $l0
                    i32.store8
                    br $B23
                  end
                  get_local $p0
                  i32.const 2057
                  i32.add
                  i32.const 0
                  i32.store16 align=1
                  br $B23
                end
                get_local $p0
                i32.const 2058
                i32.add
                i32.const 0
                i32.store8
                get_local $p0
                i32.const 2056
                i32.add
                i32.const 0
                i32.store16
              end
              get_local $p0
              i32.const 2048
              i32.add
              i64.const 0
              i64.store align=4
              get_local $p0
              i32.const 2068
              i32.add
              get_local $l2
              call $get_se_golomb_long
              i32.store
              block $B26
                get_local $p0
                i32.load offset=204
                tee_local $l0
                i32.load8_u offset=36
                if $I27
                  get_local $p0
                  i32.const 2072
                  i32.add
                  get_local $l2
                  call $get_se_golomb_long
                  i32.store
                  get_local $l2
                  call $get_se_golomb_long
                  set_local $l1
                  get_local $p0
                  i32.load offset=204
                  set_local $l0
                  br $B26
                end
                i32.const 0
                set_local $l1
                get_local $p0
                i32.const 2072
                i32.add
                i32.const 0
                i32.store
              end
              get_local $p0
              i32.const 2076
              i32.add
              get_local $l1
              i32.store
              block $B28
                get_local $l0
                i32.load8_u offset=1631
                i32.eqz
                if $I29
                  i32.const 0
                  set_local $l1
                  br $B28
                end
                get_local $l2
                call $get_bits1
                set_local $l1
                get_local $p0
                i32.load offset=204
                set_local $l0
              end
              get_local $p0
              i32.const 2080
              i32.add
              get_local $l1
              i32.store8
              block $B30
                get_local $p0
                i32.const 2088
                i32.add
                block $B31 (result i32)
                  get_local $l0
                  i32.load8_u offset=55
                  if $I32
                    block $B33
                      get_local $l0
                      i32.load8_u offset=56
                      if $I34
                        get_local $l2
                        call $get_bits1
                        br_if $B33
                        get_local $p0
                        i32.load offset=204
                        set_local $l0
                      end
                      get_local $p0
                      i32.const 2061
                      i32.add
                      get_local $l0
                      i32.load8_u offset=57
                      i32.store8
                      get_local $p0
                      i32.const 2084
                      i32.add
                      get_local $l0
                      i32.load offset=60
                      i32.store
                      get_local $l0
                      i32.load offset=64
                      br $B31
                    end
                    get_local $p0
                    i32.const 2061
                    i32.add
                    get_local $l2
                    call $get_bits1
                    tee_local $l0
                    i32.store8
                    get_local $l0
                    i32.const 255
                    i32.and
                    br_if $B30
                    get_local $p0
                    i32.const 2084
                    i32.add
                    get_local $l2
                    call $get_se_golomb_long
                    i32.const 1
                    i32.shl
                    i32.store
                    get_local $l2
                    call $get_se_golomb_long
                    i32.const 1
                    i32.shl
                    br $B31
                  end
                  get_local $p0
                  i32.const 2084
                  i32.add
                  i32.const 0
                  i32.store
                  get_local $p0
                  i32.const 2061
                  i32.add
                  i32.const 0
                  i32.store8
                  i32.const 0
                end
                i32.store
              end
              block $B35
                get_local $p0
                i32.load offset=204
                i32.load8_u offset=54
                tee_local $l0
                i32.eqz
                br_if $B35
                block $B36
                  get_local $p0
                  i32.const 2056
                  i32.add
                  i32.load8_u
                  br_if $B36
                  get_local $p0
                  i32.const 2057
                  i32.add
                  i32.load8_u
                  br_if $B36
                  get_local $p0
                  i32.const 2061
                  i32.add
                  i32.load8_u
                  br_if $B35
                end
                get_local $l2
                call $get_bits1
                set_local $l0
              end
              get_local $p0
              i32.const 2062
              i32.add
              get_local $l0
              i32.store8
              br $B11
            end
            get_local $p0
            i32.load8_u offset=156
            i32.eqz
            br_if $B2
          end
          get_local $p0
          i32.const 2108
          i32.add
          i32.const 0
          i32.store
          block $B37
            get_local $p0
            i32.load offset=204
            tee_local $l0
            i32.load8_u offset=42
            i32.eqz
            if $I38
              get_local $l0
              i32.load8_u offset=43
              i32.eqz
              br_if $B37
            end
            get_local $p0
            get_local $l2
            call $get_ue_golomb_long
            tee_local $l0
            i32.store offset=2108
            get_local $l0
            i32.const 1
            i32.ge_s
            if $I39
              get_local $l2
              call $get_ue_golomb_long
              set_local $l3
              get_local $p0
              i32.const 2096
              i32.add
              tee_local $l0
              call $av_freep
              get_local $p0
              i32.const 2100
              i32.add
              tee_local $l1
              call $av_freep
              get_local $p0
              i32.const 2104
              i32.add
              tee_local $l4
              call $av_freep
              get_local $l0
              get_local $p0
              i32.load offset=2108
              i32.const 4
              call $av_malloc_array
              i32.store
              get_local $l1
              get_local $p0
              i32.load offset=2108
              i32.const 4
              call $av_malloc_array
              i32.store
              get_local $l4
              get_local $p0
              i32.load offset=2108
              i32.const 4
              call $av_malloc_array
              tee_local $l4
              i32.store
              block $B40
                get_local $l0
                i32.load
                i32.eqz
                get_local $l4
                i32.eqz
                i32.or
                br_if $B40
                get_local $l1
                i32.load
                i32.eqz
                br_if $B40
                get_local $p0
                i32.load offset=2108
                i32.const 1
                i32.ge_s
                if $I41
                  get_local $l3
                  i32.const 1
                  i32.add
                  tee_local $l0
                  i32.const 15
                  i32.and
                  set_local $l3
                  get_local $l0
                  i32.const 4
                  i32.shr_s
                  tee_local $l1
                  i32.const 1
                  get_local $l1
                  i32.const 1
                  i32.gt_s
                  select
                  set_local $l5
                  i32.const 0
                  set_local $l4
                  get_local $l0
                  i32.const 16
                  i32.lt_s
                  set_local $l6
                  loop $L42
                    i32.const 0
                    set_local $l0
                    i32.const 0
                    set_local $l1
                    get_local $l6
                    i32.eqz
                    if $I43
                      loop $L44
                        get_local $l2
                        i32.const 16
                        call $get_bits
                        get_local $l0
                        i32.const 16
                        i32.shl
                        i32.add
                        set_local $l0
                        get_local $l1
                        i32.const 1
                        i32.add
                        tee_local $l1
                        get_local $l5
                        i32.ne
                        br_if $L44
                      end
                    end
                    get_local $l3
                    if $I45
                      get_local $l2
                      get_local $l3
                      call $get_bits
                      get_local $l0
                      get_local $l3
                      i32.shl
                      i32.add
                      set_local $l0
                    end
                    get_local $p0
                    i32.load offset=2096
                    get_local $l4
                    i32.const 2
                    i32.shl
                    i32.add
                    get_local $l0
                    i32.const 1
                    i32.add
                    i32.store
                    get_local $l4
                    i32.const 1
                    i32.add
                    tee_local $l4
                    get_local $p0
                    i32.load offset=2108
                    i32.lt_s
                    br_if $L42
                  end
                end
                block $B46
                  get_local $p0
                  i32.load8_u offset=141
                  i32.const 2
                  i32.lt_u
                  br_if $B46
                  get_local $p0
                  i32.load offset=204
                  tee_local $l0
                  i32.load offset=48
                  i32.const 1
                  i32.le_s
                  if $I47
                    get_local $l0
                    i32.load offset=44
                    i32.const 2
                    i32.lt_s
                    br_if $B46
                  end
                  get_local $p0
                  i32.const 1
                  i32.store8 offset=141
                  get_local $p0
                  i32.const 0
                  i32.store offset=4368
                  br $B37
                end
                get_local $p0
                i32.const 0
                i32.store offset=4368
                br $B37
              end
              get_local $p0
              i32.const 0
              i32.store offset=2108
              i32.const -48
              return
            end
            get_local $p0
            i32.const 0
            i32.store offset=4368
          end
          get_local $p0
          i32.load offset=204
          tee_local $l1
          i32.load8_u offset=1628
          if $I48
            get_local $l2
            call $get_ue_golomb_long
            tee_local $l1
            i64.extend_u/i32
            i64.const 3
            i64.shl
            get_local $l2
            call $get_bits_left
            i64.extend_s/i32
            i64.gt_s
            br_if $B2
            get_local $l1
            if $I49
              i32.const 0
              set_local $l0
              loop $L50
                get_local $l2
                i32.const 8
                call $skip_bits
                get_local $l0
                i32.const 1
                i32.add
                tee_local $l0
                get_local $l1
                i32.ne
                br_if $L50
              end
            end
            get_local $p0
            i32.load offset=204
            set_local $l1
          end
          get_local $p0
          i32.const 2112
          i32.add
          get_local $l1
          i32.load offset=16
          get_local $p0
          i32.const 2068
          i32.add
          i32.load
          i32.add
          i32.const 26
          i32.add
          tee_local $l3
          i32.store8
          get_local $l3
          i32.const 24
          i32.shl
          tee_local $l0
          i32.const 855638016
          i32.gt_s
          br_if $B2
          i32.const 0
          get_local $p0
          i32.load offset=200
          i32.load offset=13192
          i32.sub
          get_local $l0
          i32.const 24
          i32.shr_s
          i32.gt_s
          br_if $B2
          get_local $p0
          i32.const 2500
          i32.add
          get_local $p0
          i32.const 1432
          i32.add
          i32.load
          tee_local $l0
          i32.store
          get_local $l0
          i32.eqz
          if $I51
            get_local $p0
            i32.load8_u offset=1449
            br_if $B2
          end
          i32.const -1094995529
          set_local $l0
          get_local $l2
          call $get_bits_left
          i32.const 0
          i32.lt_s
          br_if $B7
          get_local $p0
          i32.load offset=136
          tee_local $l2
          get_local $p0
          i32.load8_u offset=1449
          i32.eqz
          i32.store8 offset=203
          get_local $l1
          i32.load8_u offset=22
          i32.eqz
          if $I52
            get_local $l2
            get_local $l3
            i32.store8 offset=272
          end
          get_local $p0
          i32.const 1
          i32.store8 offset=156
          i32.const 0
          set_local $l0
          get_local $l2
          i32.const 0
          i32.store16 offset=302
        end
        get_local $l0
        return
      end
      call $abort
      unreachable
    end
    i32.const -1094995529)
  (func $hevc_frame_start (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32)
    get_local $p0
    i32.load offset=136
    set_local $l2
    get_local $p0
    i32.load offset=200
    tee_local $l0
    i32.load offset=13120
    set_local $l3
    get_local $l0
    i32.load offset=13124
    set_local $l4
    get_local $l0
    i32.load offset=13064
    set_local $l0
    get_local $p0
    i32.load offset=4320
    i32.const 0
    get_local $p0
    i32.load offset=2600
    get_local $p0
    i32.load offset=2596
    i32.mul
    call $memset
    drop
    get_local $p0
    i32.load offset=4324
    i32.const 0
    get_local $p0
    i32.load offset=2600
    get_local $p0
    i32.load offset=2596
    i32.mul
    call $memset
    drop
    get_local $p0
    i32.load offset=4344
    i32.const 0
    get_local $p0
    i32.load offset=200
    tee_local $l1
    i32.load offset=13152
    get_local $l1
    i32.load offset=13148
    i32.mul
    call $memset
    drop
    get_local $p0
    i32.load offset=4348
    i32.const 0
    get_local $p0
    i32.load offset=200
    tee_local $l1
    i32.load offset=13160
    i32.const 1
    i32.add
    get_local $l1
    i32.load offset=13156
    i32.const 1
    i32.add
    i32.mul
    call $memset
    drop
    get_local $p0
    i32.load offset=4328
    i32.const 255
    get_local $l3
    get_local $l0
    i32.shr_s
    i32.const 2
    i32.shl
    i32.const 4
    i32.add
    get_local $l4
    get_local $l0
    i32.shr_s
    i32.const 1
    i32.add
    i32.mul
    call $memset
    drop
    get_local $p0
    i32.const 0
    i32.store offset=2604
    get_local $p0
    get_local $p0
    i32.load offset=2512
    i32.store offset=4416
    get_local $p0
    i32.load offset=204
    tee_local $l0
    i32.load8_u offset=42
    if $I0
      get_local $l2
      get_local $l0
      i32.load offset=1648
      i32.load
      get_local $p0
      i32.load offset=200
      i32.load offset=13080
      i32.shl
      i32.store offset=312
    end
    block $B1
      get_local $p0
      get_local $p0
      i32.const 160
      i32.add
      get_local $p0
      i32.load offset=2572
      call $ff_hevc_set_new_ref
      tee_local $l0
      i32.const 0
      i32.lt_s
      br_if $B1
      get_local $p0
      i32.load offset=2520
      i32.load
      get_local $p0
      i32.load offset=2512
      i32.const -8
      i32.and
      i32.const 16
      i32.eq
      i32.store offset=80
      get_local $p0
      i32.load offset=160
      i32.const 3
      get_local $p0
      i32.const 1440
      i32.add
      i32.load
      i32.sub
      i32.store offset=84
      get_local $p0
      i32.load offset=164
      call $av_frame_unref
      get_local $p0
      get_local $p0
      i32.load offset=164
      i32.const 0
      call $ff_hevc_output_frame
      tee_local $l0
      i32.const 0
      i32.lt_s
      br_if $B1
      get_local $p0
      i32.load offset=4
      drop
      i32.const 0
      return
    end
    block $B2
      get_local $p0
      i32.load offset=2520
      i32.eqz
      br_if $B2
      get_local $p0
      i32.load8_u offset=140
      i32.const 1
      i32.ne
      br_if $B2
    end
    get_local $p0
    i32.const 0
    i32.store offset=2520
    get_local $l0)
  (func $hls_slice_data (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l0
    set_global $g0
    get_local $l0
    i64.const 4294967296
    i64.store offset=8 align=4
    get_local $p0
    i32.load offset=4
    tee_local $p0
    i32.const 1
    get_local $l0
    i32.const 8
    i32.add
    get_local $l0
    i32.const 1
    i32.const 4
    get_local $p0
    i32.load offset=816
    call_indirect (type $t18)
    drop
    get_local $l0
    i32.load
    set_local $p0
    get_local $l0
    i32.const 16
    i32.add
    set_global $g0
    get_local $p0)
  (func $set_sps (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l4
    set_global $g0
    get_local $p0
    call $pic_arrays_free
    block $B0
      get_local $p0
      get_local $p1
      call $pic_arrays_init
      tee_local $l0
      i32.const 0
      i32.ge_s
      if $I1
        get_local $p0
        i32.load offset=4
        tee_local $l0
        get_local $p1
        i32.load offset=13120
        i32.store offset=124
        get_local $l0
        get_local $p1
        i32.load offset=13124
        i32.store offset=128
        get_local $l0
        get_local $p1
        i32.load offset=12
        i32.store offset=116
        get_local $l0
        get_local $p1
        i32.load offset=16
        i32.store offset=120
        get_local $l0
        get_local $p1
        i32.load offset=60
        i32.store offset=136
        get_local $l0
        get_local $p1
        i32.load offset=72
        i32.const 12
        i32.mul
        get_local $p1
        i32.add
        i32.load offset=68
        i32.store offset=172
        get_local $l4
        get_local $p1
        i64.load offset=160 align=4
        i64.store offset=8
        get_local $p0
        i32.load offset=4
        tee_local $l0
        block $B2 (result i32)
          i32.const 1
          get_local $p1
          i32.load offset=176
          i32.eqz
          br_if $B2
          drop
          i32.const 2
          i32.const 1
          get_local $p1
          i32.load offset=184
          select
        end
        i32.store offset=392
        get_local $l0
        block $B3 (result i32)
          get_local $p1
          i32.load offset=188
          if $I4
            get_local $l0
            get_local $p1
            i32.load8_u offset=192
            i32.store offset=380
            get_local $l0
            get_local $p1
            i32.load8_u offset=193
            i32.store offset=384
            get_local $p1
            i32.load8_u offset=194
            br $B3
          end
          get_local $l0
          i64.const 8589934594
          i64.store offset=380 align=4
          i32.const 2
        end
        i32.store offset=388
        get_local $p0
        i32.const 2608
        i32.add
        get_local $p1
        i32.load offset=52
        call $ff_hevc_dsp_init
        get_local $p1
        i32.load8_u offset=12941
        if $I5
          get_local $p0
          i32.load offset=200
          tee_local $l0
          i32.load offset=4
          set_local $l1
          get_local $p0
          i32.const 1
          get_local $l0
          i32.load offset=13080
          i32.shl
          i32.const 2
          i32.add
          tee_local $l2
          get_local $l2
          i32.mul
          get_local $l0
          i32.load offset=56
          i32.shl
          call $av_malloc
          i32.store offset=168
          i32.const 3
          i32.const 1
          get_local $l1
          select
          set_local $l2
          i32.const 0
          set_local $l0
          loop $L6
            get_local $p0
            i32.load offset=200
            tee_local $l1
            get_local $l0
            i32.const 2
            i32.shl
            tee_local $l3
            i32.add
            tee_local $l5
            i32.const 13180
            i32.add
            i32.load
            set_local $l6
            get_local $l1
            i32.load offset=13124
            set_local $l7
            get_local $p0
            get_local $l3
            i32.add
            tee_local $l3
            get_local $l1
            i32.load offset=13132
            get_local $l1
            i32.load offset=13120
            get_local $l5
            i32.const 13168
            i32.add
            i32.load
            i32.shr_s
            i32.mul
            i32.const 1
            i32.shl
            get_local $l1
            i32.load offset=56
            i32.shl
            call $av_malloc
            i32.store offset=172
            get_local $l3
            get_local $p0
            i32.load offset=200
            tee_local $l1
            i32.load offset=13128
            get_local $l7
            get_local $l6
            i32.shr_s
            i32.mul
            i32.const 1
            i32.shl
            get_local $l1
            i32.load offset=56
            i32.shl
            call $av_malloc
            i32.store offset=184
            get_local $l0
            i32.const 1
            i32.add
            tee_local $l0
            get_local $l2
            i32.ne
            br_if $L6
          end
        end
        get_local $p0
        get_local $p1
        i32.store offset=200
        get_local $p0
        get_local $p0
        get_local $p1
        i32.load
        i32.const 2
        i32.shl
        i32.add
        i32.load offset=208
        i32.load offset=4
        i32.store offset=196
        i32.const 0
        set_local $l0
        br $B0
      end
      get_local $p0
      call $pic_arrays_free
      get_local $p0
      i32.const 0
      i32.store offset=200
    end
    get_local $l4
    i32.const 16
    i32.add
    set_global $g0
    get_local $l0)
  (func $hls_decode_entry (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32)
    get_local $p0
    i32.load offset=60
    tee_local $l0
    i32.load offset=200
    tee_local $l1
    i32.load offset=13080
    set_local $l3
    get_local $l0
    i32.const 1449
    i32.add
    i32.load8_u
    set_local $l2
    block $B0
      block $B1
        get_local $l0
        i32.load offset=204
        tee_local $l4
        i32.load offset=1668
        get_local $l0
        i32.const 2500
        i32.add
        i32.load
        i32.const 2
        i32.shl
        i32.add
        i32.load
        tee_local $p1
        i32.eqz
        if $I2
          i32.const -1094995529
          set_local $p0
          get_local $l2
          i32.eqz
          br_if $B1
          br $B0
        end
        get_local $l2
        i32.eqz
        br_if $B1
        i32.const -1094995529
        set_local $p0
        get_local $l0
        i32.load offset=4328
        get_local $l4
        i32.load offset=1672
        get_local $p1
        i32.const 2
        i32.shl
        i32.add
        i32.const 4
        i32.sub
        i32.load
        i32.const 2
        i32.shl
        i32.add
        i32.load
        get_local $l0
        i32.const 1436
        i32.add
        i32.load
        i32.ne
        br_if $B0
      end
      i32.const 1
      get_local $l3
      i32.shl
      set_local $l3
      i32.const 0
      set_local $p0
      block $B3
        get_local $l1
        i32.load offset=13136
        get_local $p1
        i32.le_s
        if $I4
          i32.const 0
          set_local $l2
          br $B3
        end
        get_local $l3
        i32.const 1
        i32.sub
        set_local $l6
        loop $L5
          get_local $l0
          get_local $l0
          i32.load offset=204
          i32.load offset=1672
          get_local $p1
          i32.const 2
          i32.shl
          i32.add
          i32.load
          tee_local $l4
          get_local $l4
          get_local $l6
          get_local $l1
          i32.load offset=13120
          i32.add
          get_local $l1
          i32.load offset=13080
          tee_local $l1
          i32.shr_s
          tee_local $p0
          i32.div_s
          tee_local $l2
          get_local $p0
          i32.mul
          i32.sub
          get_local $l1
          i32.shl
          tee_local $p0
          get_local $l2
          get_local $l1
          i32.shl
          tee_local $l2
          get_local $p1
          call $hls_decode_neighbour
          get_local $l0
          get_local $p1
          call $ff_hevc_cabac_init
          get_local $l0
          get_local $p0
          get_local $l0
          i32.load offset=200
          i32.load offset=13080
          tee_local $l1
          i32.shr_s
          get_local $l2
          get_local $l1
          i32.shr_s
          call $hls_sao_param
          get_local $l0
          i32.load offset=2508
          get_local $l4
          i32.const 3
          i32.shl
          i32.add
          tee_local $l1
          get_local $l0
          i32.load offset=2084
          i32.store
          get_local $l1
          get_local $l0
          i32.load offset=2088
          i32.store offset=4
          get_local $l4
          get_local $l0
          i32.load offset=4352
          i32.add
          get_local $l0
          i32.load8_u offset=2062
          i32.store8
          get_local $l0
          get_local $p0
          get_local $l2
          get_local $l0
          i32.load offset=200
          i32.load offset=13080
          i32.const 0
          call $hls_coding_quadtree
          tee_local $l5
          i32.const -1
          i32.le_s
          if $I6
            get_local $l0
            i32.load offset=4328
            get_local $l4
            i32.const 2
            i32.shl
            i32.add
            i32.const -1
            i32.store
            get_local $l5
            return
          end
          get_local $l0
          get_local $p1
          i32.const 1
          i32.add
          tee_local $p1
          call $ff_hevc_save_states
          get_local $l0
          get_local $p0
          get_local $l2
          get_local $l3
          call $ff_hevc_hls_filters
          get_local $l0
          i32.load offset=200
          set_local $l1
          get_local $l5
          i32.eqz
          br_if $B3
          get_local $p1
          get_local $l1
          i32.load offset=13136
          i32.lt_s
          br_if $L5
        end
      end
      block $B7
        get_local $l1
        i32.load offset=13120
        get_local $p0
        get_local $l3
        i32.add
        i32.gt_s
        br_if $B7
        get_local $l1
        i32.load offset=13124
        get_local $l2
        get_local $l3
        i32.add
        i32.gt_s
        br_if $B7
        get_local $l0
        get_local $p0
        get_local $l2
        get_local $l3
        call $ff_hevc_hls_filter
      end
      get_local $p1
      set_local $p0
    end
    get_local $p0)
  (func $pic_arrays_init (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32)
    get_local $p1
    i32.load offset=13160
    set_local $l4
    get_local $p1
    i32.load offset=13156
    set_local $l5
    get_local $p1
    i32.load offset=13064
    set_local $l2
    get_local $p1
    i32.load offset=13132
    set_local $l1
    get_local $p1
    i32.load offset=13128
    set_local $l0
    get_local $p1
    i32.load offset=13120
    set_local $l3
    get_local $p0
    get_local $p1
    i32.load offset=13124
    tee_local $l6
    i32.const 2
    i32.shr_s
    i32.const 1
    i32.add
    i32.store offset=2600
    get_local $p0
    get_local $l3
    i32.const 2
    i32.shr_s
    i32.const 1
    i32.add
    i32.store offset=2596
    get_local $p0
    get_local $l0
    get_local $l1
    i32.mul
    tee_local $l1
    i32.const 148
    call $av_mallocz_array
    i32.store offset=2504
    get_local $p0
    get_local $l1
    i32.const 8
    call $av_mallocz_array
    tee_local $l0
    i32.store offset=2508
    block $B0 (result i32)
      block $B1
        get_local $l0
        i32.eqz
        br_if $B1
        get_local $p0
        i32.load offset=2504
        i32.eqz
        br_if $B1
        get_local $p0
        get_local $p1
        i32.load offset=13140
        get_local $p1
        i32.load offset=13144
        i32.mul
        call $av_malloc
        i32.store offset=4332
        get_local $p0
        get_local $p1
        i32.load offset=13144
        get_local $p1
        i32.load offset=13140
        call $av_malloc_array
        tee_local $l0
        i32.store offset=4336
        get_local $l0
        i32.eqz
        br_if $B1
        get_local $p0
        i32.load offset=4332
        i32.eqz
        br_if $B1
        get_local $p0
        get_local $p1
        i32.load offset=13148
        get_local $p1
        i32.load offset=13152
        call $av_malloc_array
        i32.store offset=4344
        get_local $p0
        get_local $l4
        get_local $l5
        i32.mul
        call $av_mallocz
        i32.store offset=4340
        get_local $p0
        get_local $p1
        i32.load offset=13160
        i32.const 1
        i32.add
        get_local $p1
        i32.load offset=13156
        i32.const 1
        i32.add
        i32.mul
        call $av_malloc
        tee_local $p1
        i32.store offset=4348
        get_local $p0
        i32.load offset=4340
        i32.eqz
        get_local $p1
        i32.eqz
        i32.or
        br_if $B1
        get_local $p0
        i32.load offset=4344
        i32.eqz
        br_if $B1
        get_local $p0
        get_local $l1
        call $av_malloc
        i32.store offset=4352
        get_local $p0
        get_local $l6
        get_local $l2
        i32.shr_s
        i32.const 1
        i32.add
        get_local $l3
        get_local $l2
        i32.shr_s
        i32.const 1
        i32.add
        i32.mul
        tee_local $p1
        i32.const 4
        call $av_malloc_array
        i32.store offset=4328
        get_local $p0
        get_local $p1
        i32.const 1
        call $av_malloc_array
        tee_local $p1
        i32.store offset=4316
        get_local $p1
        i32.eqz
        br_if $B1
        get_local $p0
        i32.load offset=4352
        i32.eqz
        br_if $B1
        get_local $p0
        i32.load offset=4328
        i32.eqz
        br_if $B1
        get_local $p0
        get_local $p0
        i32.load offset=2596
        get_local $p0
        i32.load offset=2600
        call $av_mallocz_array
        i32.store offset=4320
        get_local $p0
        get_local $p0
        i32.load offset=2596
        get_local $p0
        i32.load offset=2600
        call $av_mallocz_array
        tee_local $p1
        i32.store offset=4324
        get_local $p1
        i32.eqz
        br_if $B1
        i32.const 0
        get_local $p0
        i32.load offset=4320
        br_if $B0
        drop
      end
      get_local $p0
      call $pic_arrays_free
      i32.const -48
    end)
  (func $hls_decode_neighbour (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32)
    get_local $p0
    i32.load offset=136
    set_local $l0
    get_local $p0
    i32.load offset=200
    tee_local $l2
    i32.load offset=13080
    set_local $l3
    get_local $p0
    i32.load offset=4328
    tee_local $l8
    get_local $p0
    i32.load offset=204
    tee_local $l1
    i32.load offset=1672
    get_local $p3
    i32.const 2
    i32.shl
    i32.add
    i32.load
    tee_local $l6
    i32.const 2
    i32.shl
    i32.add
    tee_local $l9
    get_local $p0
    i32.const 1436
    i32.add
    i32.load
    tee_local $p0
    i32.store
    i32.const 1
    get_local $l3
    i32.shl
    set_local $l5
    block $B0
      get_local $l1
      i32.load8_u offset=43
      if $I1
        get_local $p1
        get_local $l5
        i32.const 1
        i32.sub
        get_local $p2
        i32.and
        i32.or
        i32.eqz
        if $I2
          get_local $l0
          i32.const 1
          i32.store8 offset=203
        end
        get_local $l0
        get_local $l2
        i32.load offset=13120
        i32.store offset=312
        get_local $l1
        i32.load8_u offset=42
        i32.eqz
        set_local $l4
        br $B0
      end
      get_local $l1
      i32.load8_u offset=42
      if $I3
        get_local $p3
        i32.eqz
        br_if $B0
        get_local $l1
        i32.load offset=1676
        get_local $p3
        i32.const 2
        i32.shl
        i32.add
        tee_local $l3
        i32.load
        get_local $l3
        i32.const 4
        i32.sub
        i32.load
        i32.eq
        br_if $B0
        get_local $l1
        i32.load offset=1648
        get_local $l1
        i32.load offset=1664
        get_local $p1
        get_local $l2
        i32.load offset=13080
        tee_local $l3
        i32.shr_s
        i32.const 2
        i32.shl
        i32.add
        i32.load
        i32.const 2
        i32.shl
        i32.add
        i32.load
        set_local $l7
        get_local $l0
        i32.const 1
        i32.store8 offset=203
        get_local $l0
        get_local $l7
        get_local $l3
        i32.shl
        get_local $p1
        i32.add
        i32.store offset=312
        br $B0
      end
      get_local $l0
      get_local $l2
      i32.load offset=13120
      i32.store offset=312
      i32.const 1
      set_local $l4
    end
    get_local $l6
    get_local $p0
    i32.sub
    set_local $l3
    get_local $l2
    i32.load offset=13124
    set_local $l7
    i32.const 0
    set_local $p0
    get_local $l0
    i32.const 0
    i32.store offset=31312
    get_local $l0
    get_local $l7
    get_local $p2
    get_local $l5
    i32.add
    tee_local $l5
    get_local $l5
    get_local $l7
    i32.gt_s
    select
    i32.store offset=316
    block $B4
      get_local $l0
      block $B5 (result i32)
        get_local $l4
        i32.eqz
        if $I6
          block $B7
            get_local $p1
            i32.const 1
            i32.lt_s
            br_if $B7
            get_local $l1
            i32.load offset=1676
            tee_local $l4
            get_local $p3
            i32.const 2
            i32.shl
            i32.add
            i32.load
            get_local $l4
            get_local $l6
            i32.const 1
            i32.sub
            i32.const 2
            i32.shl
            tee_local $l5
            get_local $l1
            i32.load offset=1668
            i32.add
            i32.load
            i32.const 2
            i32.shl
            i32.add
            i32.load
            i32.ne
            if $I8
              get_local $l0
              i32.const 2
              i32.store offset=31312
              i32.const 2
              set_local $p0
            end
            get_local $l9
            i32.load
            get_local $l5
            get_local $l8
            i32.add
            i32.load
            i32.eq
            br_if $B7
            get_local $l0
            get_local $p0
            i32.const 1
            i32.or
            tee_local $p0
            i32.store offset=31312
          end
          get_local $p2
          i32.const 1
          i32.lt_s
          br_if $B4
          get_local $l1
          i32.load offset=1676
          tee_local $l4
          get_local $p3
          i32.const 2
          i32.shl
          i32.add
          i32.load
          get_local $l4
          get_local $l6
          get_local $l2
          i32.load offset=13128
          i32.sub
          i32.const 2
          i32.shl
          tee_local $l5
          get_local $l1
          i32.load offset=1668
          i32.add
          i32.load
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.ne
          if $I9
            get_local $l0
            get_local $p0
            i32.const 8
            i32.or
            tee_local $p0
            i32.store offset=31312
          end
          get_local $l9
          i32.load
          get_local $l5
          get_local $l8
          i32.add
          i32.load
          i32.eq
          br_if $B4
          get_local $p0
          i32.const 4
          i32.or
          br $B5
        end
        get_local $l3
        i32.eqz
        if $I10
          get_local $l0
          i32.const 1
          i32.store offset=31312
          i32.const 1
          set_local $p0
        end
        get_local $l3
        get_local $l2
        i32.load offset=13128
        i32.ge_s
        br_if $B4
        get_local $p0
        i32.const 4
        i32.or
      end
      tee_local $p0
      i32.store offset=31312
    end
    i32.const 0
    set_local $l4
    get_local $l0
    get_local $l4
    get_local $p0
    i32.const 2
    i32.and
    i32.eqz
    get_local $p1
    i32.const 1
    i32.lt_s
    get_local $l3
    i32.const 1
    i32.lt_s
    i32.or
    select
    i32.store8 offset=308
    block $B11
      get_local $p2
      i32.const 1
      i32.ge_s
      if $I12
        get_local $l0
        get_local $p0
        i32.const 8
        i32.and
        i32.eqz
        get_local $l3
        get_local $l2
        i32.load offset=13128
        tee_local $l2
        i32.ge_s
        i32.and
        i32.store8 offset=309
        i32.const 0
        set_local $p0
        i32.const 0
        set_local $p2
        get_local $l0
        get_local $l2
        get_local $l3
        i32.const 1
        i32.add
        i32.le_s
        if $I13 (result i32)
          get_local $l1
          i32.load offset=1676
          tee_local $p2
          get_local $p3
          i32.const 2
          i32.shl
          i32.add
          i32.load
          get_local $p2
          get_local $l1
          i32.load offset=1668
          get_local $l6
          get_local $l2
          i32.sub
          i32.const 2
          i32.shl
          i32.add
          i32.load offset=4
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.eq
        else
          get_local $p2
        end
        i32.store8 offset=310
        get_local $p1
        i32.const 1
        i32.lt_s
        get_local $l2
        get_local $l3
        i32.ge_s
        i32.or
        br_if $B11
        get_local $l1
        i32.load offset=1676
        tee_local $p0
        get_local $p3
        i32.const 2
        i32.shl
        i32.add
        i32.load
        get_local $p0
        get_local $l1
        i32.load offset=1668
        get_local $l6
        get_local $l2
        i32.const -1
        i32.xor
        i32.add
        i32.const 2
        i32.shl
        i32.add
        i32.load
        i32.const 2
        i32.shl
        i32.add
        i32.load
        i32.eq
        set_local $p0
        br $B11
      end
      i32.const 0
      set_local $p0
      get_local $l0
      i32.const 0
      i32.store16 offset=309 align=1
    end
    get_local $l0
    get_local $p0
    i32.store8 offset=311)
  (func $hls_sao_param (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32) (local $l13 i32) (local $l14 i32) (local $l15 i32) (local $l16 i32) (local $l17 i32)
    get_local $p0
    i32.load offset=200
    i32.load offset=13128
    get_local $p2
    i32.mul
    get_local $p1
    i32.add
    set_local $l4
    get_local $p0
    i32.load offset=2504
    set_local $l0
    get_local $p0
    i32.load offset=136
    set_local $l2
    block $B0
      block $B1
        get_local $p0
        i32.const 2056
        i32.add
        i32.load8_u
        br_if $B1
        get_local $p0
        i32.const 2057
        i32.add
        i32.load8_u
        br_if $B1
        br $B0
      end
      block $B2 (result i32)
        i32.const 0
        get_local $p1
        i32.const 1
        i32.lt_s
        br_if $B2
        drop
        i32.const 0
        get_local $l2
        i32.load8_u offset=308
        i32.eqz
        br_if $B2
        drop
        get_local $p0
        call $ff_hevc_sao_merge_flag_decode
      end
      tee_local $l6
      get_local $p2
      i32.const 1
      i32.lt_s
      i32.or
      br_if $B0
      i32.const 0
      set_local $l6
      get_local $l2
      i32.load8_u offset=309
      i32.eqz
      if $I3
        br $B0
      end
      get_local $p0
      call $ff_hevc_sao_merge_flag_decode
      set_local $l5
    end
    i32.const 3
    i32.const 1
    get_local $p0
    i32.load offset=200
    i32.load offset=4
    select
    set_local $l16
    get_local $p2
    i32.const 1
    i32.sub
    set_local $l8
    get_local $p1
    i32.const 1
    i32.sub
    set_local $l9
    get_local $l5
    get_local $l6
    i32.or
    set_local $l10
    get_local $l0
    get_local $l4
    i32.const 148
    i32.mul
    i32.add
    tee_local $l3
    set_local $l17
    i32.const 0
    set_local $l0
    loop $L4
      get_local $p0
      i32.load offset=204
      set_local $l2
      block $B5
        block $B6
          block $B7
            block $B8
              block $B9
                block $B10
                  block $B11 (result i32)
                    block $B12
                      block $B13
                        get_local $l0
                        if $I14
                          get_local $p0
                          get_local $l0
                          i32.add
                          i32.const 2056
                          i32.add
                          i32.load8_u
                          br_if $B13
                          br $B6
                        end
                        get_local $p0
                        i32.load8_u offset=2056
                        i32.eqz
                        br_if $B6
                        get_local $l2
                        i32.load8_u offset=1644
                        set_local $l13
                        br $B12
                      end
                      get_local $l2
                      i32.load8_u offset=1645
                      set_local $l13
                      get_local $l0
                      i32.const 2
                      i32.ne
                      br_if $B12
                      get_local $l3
                      get_local $l3
                      i32.load8_u offset=143
                      tee_local $l1
                      i32.store8 offset=144
                      get_local $l17
                      get_local $l3
                      i32.load offset=104
                      i32.store offset=108
                      i32.const 1
                      br $B11
                    end
                    block $B15
                      get_local $l10
                      i32.eqz
                      if $I16
                        get_local $l0
                        get_local $l3
                        i32.add
                        get_local $p0
                        call $ff_hevc_sao_type_idx_decode
                        tee_local $l1
                        i32.store8 offset=142
                        br $B15
                      end
                      get_local $l6
                      if $I17
                        get_local $l0
                        get_local $l3
                        i32.add
                        get_local $p0
                        i32.load offset=2504
                        get_local $l9
                        get_local $p0
                        i32.load offset=200
                        i32.load offset=13128
                        get_local $p2
                        i32.mul
                        i32.add
                        i32.const 148
                        i32.mul
                        i32.add
                        get_local $l0
                        i32.add
                        i32.load8_u offset=142
                        tee_local $l1
                        i32.store8 offset=142
                        br $B15
                      end
                      get_local $l5
                      i32.eqz
                      br_if $B10
                      get_local $l0
                      get_local $l3
                      i32.add
                      get_local $p0
                      i32.load offset=2504
                      get_local $p0
                      i32.load offset=200
                      i32.load offset=13128
                      get_local $l8
                      i32.mul
                      get_local $p1
                      i32.add
                      i32.const 148
                      i32.mul
                      i32.add
                      get_local $l0
                      i32.add
                      i32.load8_u offset=142
                      tee_local $l1
                      i32.store8 offset=142
                    end
                    i32.const 0
                  end
                  set_local $l7
                  get_local $l1
                  i32.const 255
                  i32.and
                  i32.eqz
                  br_if $B5
                  get_local $l0
                  get_local $l3
                  i32.add
                  tee_local $l4
                  set_local $l14
                  i32.const 0
                  set_local $l1
                  loop $L18
                    block $B19 (result i32)
                      get_local $l10
                      i32.eqz
                      if $I20
                        get_local $p0
                        call $ff_hevc_sao_offset_abs_decode
                        br $B19
                      end
                      get_local $l6
                      if $I21
                        get_local $p0
                        i32.load offset=2504
                        get_local $l9
                        get_local $p0
                        i32.load offset=200
                        i32.load offset=13128
                        get_local $p2
                        i32.mul
                        i32.add
                        i32.const 148
                        i32.mul
                        i32.add
                        get_local $l0
                        i32.const 4
                        i32.shl
                        i32.add
                        get_local $l1
                        i32.const 2
                        i32.shl
                        i32.add
                        i32.load
                        br $B19
                      end
                      i32.const 0
                      get_local $l5
                      i32.eqz
                      br_if $B19
                      drop
                      get_local $p0
                      i32.load offset=2504
                      get_local $p0
                      i32.load offset=200
                      i32.load offset=13128
                      get_local $l8
                      i32.mul
                      get_local $p1
                      i32.add
                      i32.const 148
                      i32.mul
                      i32.add
                      get_local $l0
                      i32.const 4
                      i32.shl
                      i32.add
                      get_local $l1
                      i32.const 2
                      i32.shl
                      i32.add
                      i32.load
                    end
                    set_local $l2
                    get_local $l3
                    get_local $l0
                    i32.const 4
                    i32.shl
                    tee_local $l11
                    i32.add
                    tee_local $l15
                    get_local $l1
                    i32.const 2
                    i32.shl
                    i32.add
                    get_local $l2
                    i32.store
                    get_local $l1
                    i32.const 1
                    i32.add
                    tee_local $l1
                    i32.const 4
                    i32.ne
                    br_if $L18
                  end
                  i32.const 0
                  set_local $l1
                  get_local $l14
                  i32.load8_u offset=142
                  i32.const 1
                  i32.ne
                  br_if $B9
                  loop $L22
                    block $B23
                      block $B24 (result i32)
                        i32.const 0
                        get_local $l15
                        get_local $l1
                        i32.const 2
                        i32.shl
                        tee_local $l7
                        i32.add
                        tee_local $l12
                        i32.load
                        i32.eqz
                        br_if $B24
                        drop
                        get_local $l10
                        i32.eqz
                        br_if $B23
                        get_local $l6
                        if $I25
                          get_local $p0
                          i32.load offset=2504
                          get_local $l9
                          get_local $p0
                          i32.load offset=200
                          i32.load offset=13128
                          get_local $p2
                          i32.mul
                          i32.add
                          i32.const 148
                          i32.mul
                          i32.add
                          get_local $l11
                          i32.add
                          get_local $l7
                          i32.add
                          i32.load offset=48
                          br $B24
                        end
                        i32.const 0
                        get_local $l5
                        i32.eqz
                        br_if $B24
                        drop
                        get_local $p0
                        i32.load offset=2504
                        get_local $p0
                        i32.load offset=200
                        i32.load offset=13128
                        get_local $l8
                        i32.mul
                        get_local $p1
                        i32.add
                        i32.const 148
                        i32.mul
                        i32.add
                        get_local $l11
                        i32.add
                        get_local $l7
                        i32.add
                        i32.load offset=48
                      end
                      set_local $l2
                      get_local $l12
                      get_local $l2
                      i32.store offset=48
                      get_local $l1
                      i32.const 1
                      i32.add
                      tee_local $l1
                      i32.const 4
                      i32.ne
                      br_if $L22
                      get_local $l10
                      i32.eqz
                      br_if $B8
                      get_local $l6
                      if $I26
                        get_local $l4
                        get_local $p0
                        i32.load offset=2504
                        get_local $l9
                        get_local $p0
                        i32.load offset=200
                        i32.load offset=13128
                        get_local $p2
                        i32.mul
                        i32.add
                        i32.const 148
                        i32.mul
                        i32.add
                        get_local $l0
                        i32.add
                        i32.load8_u offset=96
                        i32.store8 offset=96
                        br $B7
                      end
                      get_local $l5
                      if $I27
                        get_local $l4
                        get_local $p0
                        i32.load offset=2504
                        get_local $p0
                        i32.load offset=200
                        i32.load offset=13128
                        get_local $l8
                        i32.mul
                        get_local $p1
                        i32.add
                        i32.const 148
                        i32.mul
                        i32.add
                        get_local $l0
                        i32.add
                        i32.load8_u offset=96
                        i32.store8 offset=96
                        br $B7
                      end
                      get_local $l4
                      i32.const 0
                      i32.store8 offset=96
                      br $B7
                    end
                    get_local $l12
                    get_local $p0
                    call $ff_hevc_sao_offset_sign_decode
                    i32.store offset=48
                    get_local $l1
                    i32.const 1
                    i32.add
                    tee_local $l1
                    i32.const 4
                    i32.ne
                    br_if $L22
                  end
                  br $B8
                end
                get_local $l0
                get_local $l3
                i32.add
                i32.const 0
                i32.store8 offset=142
                br $B5
              end
              get_local $l7
              br_if $B7
              get_local $l10
              i32.eqz
              if $I28
                get_local $l3
                get_local $l0
                i32.const 2
                i32.shl
                i32.add
                get_local $p0
                call $ff_hevc_sao_eo_class_decode
                i32.store offset=100
                br $B7
              end
              get_local $l6
              if $I29
                get_local $l3
                get_local $l0
                i32.const 2
                i32.shl
                tee_local $l2
                i32.add
                get_local $p0
                i32.load offset=2504
                get_local $l9
                get_local $p0
                i32.load offset=200
                i32.load offset=13128
                get_local $p2
                i32.mul
                i32.add
                i32.const 148
                i32.mul
                i32.add
                get_local $l2
                i32.add
                i32.load offset=100
                i32.store offset=100
                br $B7
              end
              get_local $l5
              if $I30
                get_local $l3
                get_local $l0
                i32.const 2
                i32.shl
                tee_local $l2
                i32.add
                get_local $p0
                i32.load offset=2504
                get_local $p0
                i32.load offset=200
                i32.load offset=13128
                get_local $l8
                i32.mul
                get_local $p1
                i32.add
                i32.const 148
                i32.mul
                i32.add
                get_local $l2
                i32.add
                i32.load offset=100
                i32.store offset=100
                br $B7
              end
              get_local $l3
              get_local $l0
              i32.const 2
              i32.shl
              i32.add
              i32.const 0
              i32.store offset=100
              br $B7
            end
            get_local $l4
            get_local $p0
            call $ff_hevc_sao_band_position_decode
            i32.store8 offset=96
          end
          i32.const 0
          set_local $l2
          get_local $l3
          get_local $l0
          i32.const 10
          i32.mul
          i32.add
          tee_local $l11
          i32.const 0
          i32.store16 offset=112
          loop $L31
            get_local $l11
            get_local $l2
            tee_local $l4
            i32.const 1
            i32.add
            tee_local $l2
            i32.const 1
            i32.shl
            i32.add
            tee_local $l7
            i32.const 112
            i32.add
            get_local $l15
            get_local $l4
            i32.const 2
            i32.shl
            i32.add
            tee_local $l12
            i32.load
            tee_local $l1
            i32.store16
            block $B32
              block $B33
                get_local $l14
                i32.load8_u offset=142
                i32.const 2
                i32.eq
                if $I34
                  get_local $l4
                  i32.const 2
                  i32.ge_u
                  br_if $B33
                  br $B32
                end
                get_local $l12
                i32.load offset=48
                i32.eqz
                br_if $B32
              end
              get_local $l7
              i32.const 0
              get_local $l1
              i32.sub
              tee_local $l1
              i32.store16 offset=112
            end
            get_local $l7
            get_local $l1
            i32.const 16
            i32.shl
            i32.const 16
            i32.shr_s
            get_local $l13
            i32.shl
            i32.store16 offset=112
            get_local $l2
            i32.const 4
            i32.ne
            br_if $L31
          end
          br $B5
        end
        get_local $l0
        get_local $l3
        i32.add
        i32.const 0
        i32.store8 offset=142
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      get_local $l16
      i32.ne
      br_if $L4
    end)
  (func $hls_coding_quadtree (type $t13) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32)
    get_local $p0
    i32.load offset=204
    tee_local $l1
    i32.load offset=24
    set_local $l8
    get_local $p0
    i32.load offset=200
    tee_local $l0
    i32.load offset=13080
    set_local $l9
    get_local $p0
    i32.load offset=136
    tee_local $l2
    get_local $p4
    i32.store offset=31232
    block $B0
      block $B1
        i32.const 1
        get_local $p3
        i32.shl
        tee_local $l3
        get_local $p1
        i32.add
        tee_local $l4
        get_local $l0
        i32.load offset=13120
        i32.gt_s
        br_if $B1
        get_local $l0
        i32.load offset=13124
        get_local $p2
        get_local $l3
        i32.add
        i32.lt_s
        br_if $B1
        get_local $l0
        i32.load offset=13064
        get_local $p3
        i32.ge_u
        br_if $B1
        get_local $p0
        get_local $p4
        get_local $p1
        get_local $p2
        call $ff_hevc_split_coding_unit_flag_decode
        set_local $l0
        get_local $p0
        i32.load offset=204
        set_local $l1
        br $B0
      end
      get_local $l0
      i32.load offset=13064
      get_local $p3
      i32.lt_u
      set_local $l0
    end
    block $B2
      get_local $l1
      i32.load8_u offset=22
      i32.eqz
      br_if $B2
      get_local $p0
      i32.load offset=200
      i32.load offset=13080
      get_local $l1
      i32.load offset=24
      i32.sub
      get_local $p3
      i32.gt_u
      br_if $B2
      get_local $l2
      i32.const 0
      i32.store offset=280
      get_local $l2
      i32.const 0
      i32.store8 offset=300
    end
    block $B3
      get_local $p0
      i32.const 2080
      i32.add
      i32.load8_u
      i32.eqz
      br_if $B3
      get_local $p0
      i32.load offset=200
      i32.load offset=13080
      get_local $l1
      i32.load8_u offset=1632
      i32.sub
      get_local $p3
      i32.gt_u
      br_if $B3
      get_local $l2
      i32.const 0
      i32.store8 offset=301
    end
    block $B4
      get_local $l0
      if $I5
        i32.const 0
        set_local $l0
        get_local $p0
        get_local $p1
        get_local $p2
        get_local $p3
        i32.const 1
        i32.sub
        tee_local $l5
        get_local $p4
        i32.const 1
        i32.add
        tee_local $l6
        call $hls_coding_quadtree
        tee_local $p3
        i32.const 0
        i32.lt_s
        if $I6
          get_local $p3
          return
        end
        get_local $l3
        i32.const 1
        i32.shr_s
        tee_local $l7
        get_local $p2
        i32.add
        set_local $p4
        get_local $p1
        get_local $l7
        i32.add
        set_local $l1
        block $B7
          get_local $p3
          i32.eqz
          br_if $B7
          block $B8 (result i32)
            get_local $p0
            i32.load offset=200
            tee_local $p3
            i32.load offset=13120
            get_local $l1
            i32.gt_s
            if $I9
              get_local $p0
              get_local $l1
              get_local $p2
              get_local $l5
              get_local $l6
              call $hls_coding_quadtree
              tee_local $p3
              i32.const 0
              i32.lt_s
              if $I10
                get_local $p3
                return
              end
              get_local $p3
              i32.eqz
              br_if $B7
              get_local $p0
              i32.load offset=200
              set_local $p3
            end
            get_local $p3
            i32.load offset=13124
            get_local $p4
            i32.gt_s
          end
          if $I11
            get_local $p0
            get_local $p1
            get_local $p4
            get_local $l5
            get_local $l6
            call $hls_coding_quadtree
            tee_local $p1
            i32.const 0
            i32.lt_s
            if $I12
              get_local $p1
              return
            end
            get_local $p1
            i32.eqz
            br_if $B7
            get_local $p0
            i32.load offset=200
            set_local $p3
          end
          i32.const 1
          set_local $l0
          get_local $l1
          get_local $p3
          i32.load offset=13120
          i32.ge_s
          br_if $B7
          get_local $p4
          get_local $p3
          i32.load offset=13124
          i32.ge_s
          br_if $B7
          get_local $p0
          get_local $l1
          get_local $p4
          get_local $l5
          get_local $l6
          call $hls_coding_quadtree
          tee_local $l0
          i32.const 0
          i32.lt_s
          br_if $B4
        end
        get_local $l4
        i32.const -1
        get_local $l9
        get_local $l8
        i32.sub
        i32.shl
        i32.const -1
        i32.xor
        tee_local $p1
        i32.and
        get_local $p2
        get_local $l3
        i32.add
        get_local $p1
        i32.and
        i32.or
        i32.eqz
        if $I13
          get_local $l2
          get_local $l2
          i32.load8_s offset=272
          i32.store offset=276
        end
        get_local $l0
        i32.eqz
        if $I14
          i32.const 0
          return
        end
        i32.const 1
        set_local $l0
        get_local $p0
        i32.load offset=200
        tee_local $p0
        i32.load offset=13120
        get_local $l1
        get_local $l7
        i32.add
        i32.gt_s
        br_if $B4
        get_local $p0
        i32.load offset=13124
        get_local $p4
        get_local $l7
        i32.add
        i32.gt_s
        return
      end
      get_local $p0
      get_local $p1
      get_local $p2
      get_local $p3
      call $hls_coding_unit
      tee_local $l0
      i32.const 0
      i32.lt_s
      br_if $B4
      get_local $l4
      i32.const -1
      get_local $p0
      i32.load offset=200
      tee_local $p1
      i32.load offset=13080
      i32.shl
      i32.const -1
      i32.xor
      tee_local $p3
      i32.and
      if $I15
        i32.const 1
        set_local $l0
        get_local $l4
        get_local $p1
        i32.load offset=13120
        i32.lt_s
        br_if $B4
      end
      get_local $p2
      get_local $l3
      i32.add
      tee_local $p2
      get_local $p3
      i32.and
      if $I16
        i32.const 1
        set_local $l0
        get_local $p2
        get_local $p1
        i32.load offset=13124
        i32.lt_s
        br_if $B4
      end
      get_local $p0
      call $ff_hevc_end_of_slice_flag_decode
      i32.eqz
      set_local $l0
    end
    get_local $l0)
  (func $hls_coding_unit (type $t9) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32)
    get_local $p0
    i32.load offset=204
    i32.load offset=24
    set_local $l6
    get_local $p0
    i32.load offset=200
    tee_local $l0
    i32.load offset=13080
    set_local $l7
    get_local $l0
    i32.load offset=13140
    set_local $l4
    get_local $l0
    i32.load offset=13064
    set_local $l0
    get_local $p0
    i32.load offset=136
    tee_local $l1
    i32.const 31252
    i32.add
    i32.const 1
    i32.store8
    get_local $l1
    i32.const 31240
    i32.add
    get_local $p2
    i32.store
    get_local $l1
    get_local $p1
    i32.store offset=31236
    get_local $l1
    i32.const 31244
    i32.add
    i64.const 1
    i64.store align=4
    get_local $l1
    i32.const 31253
    i32.add
    i32.const 0
    i32.store16 align=1
    get_local $l4
    get_local $p2
    get_local $l0
    i32.shr_s
    i32.mul
    get_local $p1
    get_local $l0
    i32.shr_s
    i32.add
    tee_local $l3
    get_local $p0
    i32.load offset=4332
    i32.add
    i32.const 0
    i32.store8
    get_local $l1
    i32.const 31268
    i32.add
    i32.const 16843009
    i32.store align=1
    i32.const 1
    get_local $p3
    i32.shl
    tee_local $l5
    get_local $l0
    i32.shr_s
    set_local $l2
    block $B0
      get_local $p0
      i32.load offset=204
      i32.load8_u offset=40
      if $I1
        get_local $l1
        i32.const 31256
        i32.add
        get_local $p0
        i32.load offset=136
        tee_local $l0
        i32.const 224
        i32.add
        get_local $l0
        i32.const 5
        i32.add
        call $get_cabac
        tee_local $l0
        i32.store8
        get_local $l0
        i32.const 255
        i32.and
        i32.eqz
        br_if $B0
        get_local $p0
        get_local $p1
        get_local $p2
        get_local $p3
        call $set_deblocking_bypass
        br $B0
      end
      get_local $l1
      i32.const 31256
      i32.add
      i32.const 0
      i32.store8
    end
    get_local $l2
    i32.const 1
    i32.ge_s
    if $I2
      get_local $l3
      set_local $l0
      loop $L3
        get_local $p0
        i32.load offset=4332
        get_local $l0
        i32.add
        i32.const 0
        get_local $l2
        call $memset
        drop
        get_local $l0
        get_local $l4
        i32.add
        set_local $l0
        get_local $l8
        i32.const 1
        i32.add
        tee_local $l8
        get_local $l2
        i32.ne
        br_if $L3
      end
    end
    block $B4
      block $B5
        block $B6
          block $B7
            block $B8
              get_local $l1
              i32.load offset=31244
              i32.const 1
              i32.eq
              if $I9
                get_local $p0
                i32.load offset=200
                i32.load offset=13064
                get_local $p3
                i32.ne
                br_if $B8
              end
              get_local $l1
              i32.const 0
              i32.const 3
              get_local $p0
              i32.load offset=136
              tee_local $l0
              i32.const 224
              i32.add
              get_local $l0
              i32.const 13
              i32.add
              call $get_cabac
              select
              tee_local $l0
              i32.store offset=31248
              get_local $l1
              get_local $l0
              i32.const 3
              i32.eq
              get_local $l1
              i32.load offset=31244
              tee_local $l0
              i32.const 1
              i32.eq
              i32.and
              i32.store8 offset=31254
              get_local $l0
              i32.const 1
              i32.ne
              br_if $B7
            end
            block $B10
              block $B11
                get_local $l1
                i32.load offset=31248
                br_if $B11
                get_local $p0
                i32.load offset=200
                tee_local $l0
                i32.load offset=68
                i32.eqz
                br_if $B11
                get_local $l0
                i32.const 13048
                i32.add
                i32.load
                get_local $p3
                i32.gt_u
                br_if $B11
                get_local $l0
                i32.const 13052
                i32.add
                i32.load
                get_local $p3
                i32.lt_u
                br_if $B11
                get_local $l1
                get_local $p0
                call $ff_hevc_end_of_slice_flag_decode
                tee_local $l0
                i32.store8 offset=31253
                br $B10
              end
              get_local $l1
              i32.load8_u offset=31253
              set_local $l0
            end
            get_local $l0
            i32.const 255
            i32.and
            i32.eqz
            br_if $B6
            get_local $p0
            get_local $p1
            get_local $p2
            get_local $p3
            call $intra_prediction_unit_default_value
            get_local $p0
            get_local $p1
            get_local $p2
            get_local $p3
            call $hls_pcm_sample
            set_local $l0
            get_local $p0
            i32.load offset=200
            i32.const 13056
            i32.add
            i32.load8_u
            if $I12
              get_local $p0
              get_local $p1
              get_local $p2
              get_local $p3
              call $set_deblocking_bypass
            end
            get_local $l0
            i32.const 0
            i32.ge_s
            br_if $B5
            br $B4
          end
          call $abort
          unreachable
        end
        get_local $p0
        get_local $p1
        get_local $p2
        get_local $p3
        call $intra_prediction_unit
      end
      block $B13
        get_local $l1
        i32.load8_u offset=31253
        br_if $B13
        get_local $l1
        i32.load8_u offset=31252
        if $I14
          get_local $p0
          i32.load offset=200
          set_local $l0
          get_local $l1
          i32.const 31255
          i32.add
          block $B15 (result i32)
            get_local $l1
            i32.load offset=31244
            i32.const 1
            i32.eq
            if $I16
              get_local $l0
              i32.load offset=13092
              get_local $l1
              i32.load8_u offset=31254
              i32.add
              br $B15
            end
            get_local $l0
            i32.load offset=13088
          end
          i32.store8
          get_local $p0
          get_local $p1
          get_local $p2
          get_local $p1
          get_local $p2
          get_local $p1
          get_local $p2
          get_local $p3
          get_local $p3
          i32.const 0
          i32.const 0
          i32.const 2460
          i32.const 2460
          call $hls_transform_tree
          tee_local $l0
          i32.const 0
          i32.ge_s
          br_if $B13
          br $B4
        end
        get_local $p0
        i32.const 2061
        i32.add
        i32.load8_u
        br_if $B13
        get_local $p0
        get_local $p1
        get_local $p2
        get_local $p3
        call $ff_hevc_deblocking_boundary_strengths
      end
      i32.const -1
      get_local $l7
      get_local $l6
      i32.sub
      i32.shl
      set_local $p3
      block $B17
        get_local $p0
        i32.load offset=204
        i32.load8_u offset=22
        i32.eqz
        br_if $B17
        get_local $l1
        i32.load8_u offset=300
        br_if $B17
        get_local $p0
        get_local $p1
        get_local $p2
        call $ff_hevc_set_qPy
      end
      get_local $l2
      i32.const 1
      i32.ge_s
      if $I18
        i32.const 0
        set_local $l0
        loop $L19
          get_local $p0
          i32.load offset=4316
          get_local $l3
          i32.add
          get_local $l1
          i32.load8_u offset=272
          get_local $l2
          call $memset
          drop
          get_local $l3
          get_local $l4
          i32.add
          set_local $l3
          get_local $l0
          i32.const 1
          i32.add
          tee_local $l0
          get_local $l2
          i32.ne
          br_if $L19
        end
      end
      get_local $p3
      i32.const -1
      i32.xor
      tee_local $p3
      get_local $p1
      get_local $l5
      i32.add
      i32.and
      get_local $p2
      get_local $l5
      i32.add
      get_local $p3
      i32.and
      i32.or
      i32.eqz
      if $I20
        get_local $l1
        get_local $l1
        i32.load8_s offset=272
        i32.store offset=276
      end
      i32.const 1
      set_local $l3
      i32.const 0
      set_local $l0
      get_local $l5
      get_local $p0
      i32.load offset=200
      tee_local $l4
      i32.load offset=13064
      tee_local $l2
      i32.shr_s
      tee_local $p3
      i32.const 1
      i32.lt_s
      br_if $B4
      get_local $p0
      i32.load offset=4336
      get_local $p1
      get_local $l2
      i32.shr_s
      tee_local $p1
      get_local $p2
      get_local $l2
      i32.shr_s
      tee_local $p2
      get_local $l4
      i32.load offset=13140
      i32.mul
      i32.add
      i32.add
      get_local $l1
      i32.load offset=31232
      tee_local $l1
      get_local $p3
      call $memset
      drop
      get_local $p3
      i32.const 1
      i32.eq
      br_if $B4
      loop $L21
        get_local $p0
        i32.load offset=4336
        get_local $p0
        i32.load offset=200
        i32.load offset=13140
        get_local $p2
        get_local $l3
        i32.add
        i32.mul
        get_local $p1
        i32.add
        i32.add
        get_local $l1
        get_local $p3
        call $memset
        drop
        get_local $l3
        i32.const 1
        i32.add
        tee_local $l3
        get_local $p3
        i32.ne
        br_if $L21
      end
    end
    get_local $l0)
  (func $set_deblocking_bypass (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32)
    get_local $p2
    get_local $p0
    i32.load offset=200
    tee_local $l0
    i32.load offset=13084
    tee_local $l1
    i32.shr_s
    tee_local $l2
    get_local $l0
    i32.load offset=13124
    tee_local $l3
    get_local $p2
    i32.const 1
    get_local $p3
    i32.shl
    tee_local $p2
    i32.add
    tee_local $p3
    get_local $p3
    get_local $l3
    i32.gt_s
    select
    get_local $l1
    i32.shr_s
    tee_local $l3
    i32.lt_s
    if $I0
      get_local $l0
      i32.load offset=13156
      set_local $l4
      get_local $p1
      get_local $l1
      i32.shr_s
      set_local $p3
      get_local $l0
      i32.load offset=13120
      tee_local $l0
      get_local $p1
      get_local $p2
      i32.add
      tee_local $p1
      get_local $p1
      get_local $l0
      i32.gt_s
      select
      get_local $l1
      i32.shr_s
      set_local $p1
      loop $L1
        get_local $p1
        get_local $p3
        i32.gt_s
        if $I2
          get_local $l2
          get_local $l4
          i32.mul
          set_local $l0
          get_local $p3
          set_local $p2
          loop $L3
            get_local $p0
            i32.load offset=4348
            get_local $p2
            get_local $l0
            i32.add
            i32.add
            i32.const 2
            i32.store8
            get_local $p2
            i32.const 1
            i32.add
            tee_local $p2
            get_local $p1
            i32.ne
            br_if $L3
          end
        end
        get_local $l2
        i32.const 1
        i32.add
        tee_local $l2
        get_local $l3
        i32.ne
        br_if $L1
      end
    end)
  (func $intra_prediction_unit_default_value (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32)
    i32.const 1
    get_local $p3
    i32.shl
    get_local $p0
    i32.load offset=200
    tee_local $l1
    i32.load offset=13084
    tee_local $p3
    i32.shr_s
    tee_local $l0
    i32.const 1
    get_local $l0
    select
    tee_local $l0
    i32.const 1
    i32.ge_s
    if $I0
      get_local $p2
      get_local $p3
      i32.shr_s
      set_local $p2
      get_local $p1
      get_local $p3
      i32.shr_s
      set_local $p1
      get_local $l1
      i32.load offset=13156
      set_local $l1
      i32.const 0
      set_local $p3
      loop $L1
        get_local $p0
        i32.load offset=4340
        get_local $p2
        get_local $p3
        i32.add
        get_local $l1
        i32.mul
        get_local $p1
        i32.add
        i32.add
        i32.const 1
        get_local $l0
        call $memset
        drop
        get_local $p3
        i32.const 1
        i32.add
        tee_local $p3
        get_local $l0
        i32.ne
        br_if $L1
      end
    end)
  (func $hls_pcm_sample (type $t9) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32) (local $l13 i32) (local $l14 i32)
    get_global $g0
    i32.const 32
    i32.sub
    tee_local $l2
    set_global $g0
    get_local $p0
    i32.load offset=160
    tee_local $l0
    i32.load offset=32
    set_local $l4
    get_local $p0
    i32.load offset=200
    tee_local $l1
    i32.load offset=56
    set_local $l3
    get_local $l0
    i32.load
    set_local $l7
    get_local $l0
    i32.load offset=40
    set_local $l5
    get_local $l0
    i32.load offset=8
    set_local $l8
    get_local $l0
    i32.load offset=36
    set_local $l6
    get_local $l0
    i32.load offset=4
    set_local $l9
    get_local $p0
    i32.load offset=136
    i32.const 224
    i32.add
    get_local $l1
    i32.const 13045
    i32.add
    i32.load8_u
    i32.const 1
    get_local $p3
    i32.shl
    tee_local $l0
    get_local $l1
    i32.const 13176
    i32.add
    i32.load
    tee_local $l10
    i32.shr_s
    get_local $l0
    get_local $l1
    i32.const 13188
    i32.add
    i32.load
    tee_local $l11
    i32.shr_s
    i32.mul
    get_local $l0
    get_local $l1
    i32.const 13172
    i32.add
    i32.load
    tee_local $l12
    i32.shr_s
    get_local $l0
    get_local $l1
    i32.const 13184
    i32.add
    i32.load
    tee_local $l13
    i32.shr_s
    i32.mul
    i32.add
    i32.mul
    get_local $l1
    i32.load8_u offset=13044
    get_local $l0
    get_local $p3
    i32.shl
    i32.mul
    i32.add
    tee_local $l1
    i32.const 7
    i32.add
    i32.const 3
    i32.shr_s
    call $skip_bytes.1
    set_local $l14
    get_local $p0
    i32.const 2061
    i32.add
    i32.load8_u
    i32.eqz
    if $I0
      get_local $p0
      get_local $p1
      get_local $p2
      get_local $p3
      call $ff_hevc_deblocking_boundary_strengths
    end
    get_local $l2
    i32.const 8
    i32.add
    get_local $l14
    get_local $l1
    call $init_get_bits
    tee_local $p3
    i32.const 0
    i32.ge_s
    if $I1
      get_local $l7
      get_local $p2
      get_local $l4
      i32.mul
      get_local $p1
      get_local $l3
      i32.shl
      i32.add
      i32.add
      get_local $l4
      get_local $l0
      get_local $l0
      get_local $l2
      i32.const 8
      i32.add
      get_local $p0
      i32.load offset=200
      i32.load8_u offset=13044
      get_local $p0
      i32.load offset=2608
      call_indirect (type $t6)
      get_local $l9
      get_local $p2
      get_local $l13
      i32.shr_s
      get_local $l6
      i32.mul
      get_local $p1
      get_local $l12
      i32.shr_s
      get_local $l3
      i32.shl
      i32.add
      i32.add
      get_local $l6
      get_local $l0
      get_local $p0
      i32.load offset=200
      tee_local $p3
      i32.const 13172
      i32.add
      i32.load
      i32.shr_s
      get_local $l0
      get_local $p3
      i32.const 13184
      i32.add
      i32.load
      i32.shr_s
      get_local $l2
      i32.const 8
      i32.add
      get_local $p3
      i32.const 13045
      i32.add
      i32.load8_u
      get_local $p0
      i32.load offset=2608
      call_indirect (type $t6)
      get_local $l8
      get_local $p2
      get_local $l11
      i32.shr_s
      get_local $l5
      i32.mul
      get_local $p1
      get_local $l10
      i32.shr_s
      get_local $l3
      i32.shl
      i32.add
      i32.add
      get_local $l5
      get_local $l0
      get_local $p0
      i32.load offset=200
      tee_local $p1
      i32.const 13176
      i32.add
      i32.load
      i32.shr_s
      get_local $l0
      get_local $p1
      i32.const 13188
      i32.add
      i32.load
      i32.shr_s
      get_local $l2
      i32.const 8
      i32.add
      get_local $p1
      i32.const 13045
      i32.add
      i32.load8_u
      get_local $p0
      i32.load offset=2608
      call_indirect (type $t6)
      i32.const 0
      set_local $p3
    end
    get_local $l2
    i32.const 32
    i32.add
    set_global $g0
    get_local $p3)
  (func $intra_prediction_unit (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l5
    set_global $g0
    i32.const 2
    i32.const 1
    get_local $p0
    i32.load offset=136
    tee_local $l1
    i32.const 31248
    i32.add
    i32.load
    tee_local $l2
    i32.const 3
    i32.eq
    select
    set_local $l3
    i32.const 1
    get_local $p3
    i32.shl
    set_local $l4
    loop $L0
      get_local $l0
      i32.const 1
      i32.shl
      set_local $l6
      i32.const 0
      set_local $p3
      loop $L1
        get_local $l5
        i32.const 12
        i32.add
        get_local $p3
        get_local $l6
        i32.add
        i32.add
        get_local $p0
        i32.load offset=136
        tee_local $l7
        i32.const 224
        i32.add
        get_local $l7
        i32.const 17
        i32.add
        call $get_cabac
        i32.store8
        get_local $p3
        i32.const 1
        i32.add
        tee_local $p3
        get_local $l3
        i32.ne
        br_if $L1
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      get_local $l3
      i32.ne
      br_if $L0
    end
    get_local $l4
    get_local $l2
    i32.const 3
    i32.eq
    i32.shr_s
    set_local $l2
    i32.const 0
    set_local $l0
    loop $L2
      get_local $l0
      i32.const 1
      i32.shl
      set_local $l4
      get_local $l0
      get_local $l2
      i32.mul
      get_local $p2
      i32.add
      set_local $l6
      i32.const 0
      set_local $p3
      loop $L3
        block $B4
          get_local $p3
          get_local $l4
          i32.add
          tee_local $l7
          get_local $l5
          i32.const 12
          i32.add
          i32.add
          i32.load8_u
          tee_local $l8
          if $I5
            get_local $l1
            get_local $p0
            call $ff_hevc_mpm_idx_decode
            i32.store offset=31260
            br $B4
          end
          get_local $l1
          get_local $p0
          call $ff_hevc_sao_band_position_decode
          i32.store offset=31264
        end
        get_local $l1
        get_local $l7
        i32.add
        i32.const 31268
        i32.add
        get_local $p0
        get_local $p3
        get_local $l2
        i32.mul
        get_local $p1
        i32.add
        get_local $l6
        get_local $l2
        get_local $l8
        call $luma_intra_pred_mode
        i32.store8
        get_local $p3
        i32.const 1
        i32.add
        tee_local $p3
        get_local $l3
        i32.ne
        br_if $L3
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      get_local $l3
      i32.ne
      br_if $L2
    end
    i32.const 0
    set_local $p1
    block $B6
      block $B7
        block $B8
          block $B9
            get_local $p0
            i32.load offset=200
            i32.load offset=4
            br_table $B6 $B7 $B8 $B9 $B7
          end
          loop $L10
            get_local $p1
            i32.const 1
            i32.shl
            set_local $l4
            i32.const 0
            set_local $p3
            loop $L11
              get_local $l1
              get_local $p3
              get_local $l4
              i32.add
              i32.add
              tee_local $p2
              i32.const 31281
              i32.add
              get_local $p0
              call $ff_hevc_intra_chroma_pred_mode_decode
              tee_local $l0
              i32.store8
              get_local $p2
              i32.const 31268
              i32.add
              i32.load8_u
              set_local $l2
              block $B12
                get_local $l0
                i32.const 4
                i32.ne
                if $I13
                  get_local $p2
                  i32.const 31277
                  i32.add
                  set_local $p2
                  get_local $l0
                  i32.const 2468
                  i32.add
                  i32.load8_u
                  tee_local $l0
                  get_local $l2
                  i32.eq
                  if $I14
                    get_local $p2
                    i32.const 34
                    i32.store8
                    br $B12
                  end
                  get_local $p2
                  get_local $l0
                  i32.store8
                  br $B12
                end
                get_local $p2
                i32.const 31277
                i32.add
                get_local $l2
                i32.store8
              end
              get_local $p3
              i32.const 1
              i32.add
              tee_local $p3
              get_local $l3
              i32.ne
              br_if $L11
            end
            get_local $p1
            i32.const 1
            i32.add
            tee_local $p1
            get_local $l3
            i32.ne
            br_if $L10
          end
          br $B6
        end
        get_local $l1
        i32.const 31281
        i32.add
        get_local $p0
        call $ff_hevc_intra_chroma_pred_mode_decode
        tee_local $p0
        i32.store8
        get_local $l1
        i32.const 31268
        i32.add
        i32.load8_u
        set_local $p1
        get_local $l1
        i32.const 31277
        i32.add
        get_local $p0
        i32.const 4
        i32.ne
        if $I15 (result i32)
          i32.const 34
          get_local $p0
          i32.const 2468
          i32.add
          i32.load8_u
          tee_local $p0
          get_local $p0
          get_local $p1
          i32.eq
          select
        else
          get_local $p1
        end
        i32.const 255
        i32.and
        i32.const 2480
        i32.add
        i32.load8_u
        i32.store8
        br $B6
      end
      get_local $p0
      call $ff_hevc_intra_chroma_pred_mode_decode
      set_local $p0
      get_local $l1
      i32.const 31268
      i32.add
      i32.load8_u
      set_local $p1
      get_local $p0
      i32.const 4
      i32.ne
      if $I16
        get_local $p0
        i32.const 2468
        i32.add
        i32.load8_u
        tee_local $p0
        get_local $p1
        i32.eq
        if $I17
          get_local $l1
          i32.const 34
          i32.store8 offset=31277
          br $B6
        end
        get_local $l1
        get_local $p0
        i32.store8 offset=31277
        br $B6
      end
      get_local $l1
      i32.const 31277
      i32.add
      get_local $p1
      i32.store8
    end
    get_local $l5
    i32.const 16
    i32.add
    set_global $g0)
  (func $hls_transform_tree (type $t23) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (param $p8 i32) (param $p9 i32) (param $p10 i32) (param $p11 i32) (param $p12 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l0
    set_global $g0
    get_local $p0
    i32.load offset=136
    set_local $l1
    get_local $l0
    get_local $p11
    i32.load
    tee_local $l2
    i32.store offset=8
    get_local $l0
    get_local $p11
    i32.load offset=4
    tee_local $l4
    i32.store offset=12
    get_local $l0
    get_local $p12
    i32.load
    tee_local $l3
    i32.store
    get_local $l0
    get_local $p12
    i32.load offset=4
    tee_local $l5
    i32.store offset=4
    block $B0
      get_local $l1
      block $B1 (result i32)
        get_local $l1
        i32.const 31254
        i32.add
        i32.load8_u
        tee_local $p11
        if $I2
          get_local $p9
          i32.const 1
          i32.ne
          br_if $B0
          get_local $l1
          get_local $p10
          get_local $l1
          i32.add
          tee_local $p12
          i32.const 31268
          i32.add
          i32.load8_u
          i32.store offset=288
          get_local $p0
          i32.load offset=200
          i32.load offset=4
          i32.const 3
          i32.eq
          if $I3
            get_local $l1
            get_local $p12
            i32.const 31277
            i32.add
            i32.load8_u
            i32.store offset=292
            get_local $p12
            i32.const 31281
            i32.add
            br $B1
          end
          get_local $l1
          get_local $l1
          i32.const 31277
          i32.add
          i32.load8_u
          i32.store offset=292
          get_local $l1
          i32.const 31281
          i32.add
          br $B1
        end
        get_local $l1
        get_local $l1
        i32.const 31268
        i32.add
        i32.load8_u
        i32.store offset=288
        get_local $l1
        get_local $l1
        i32.const 31277
        i32.add
        i32.load8_u
        i32.store offset=292
        get_local $l1
        i32.const 31281
        i32.add
      end
      i32.load8_u
      i32.store offset=296
    end
    block $B4 (result i32)
      block $B5
        get_local $p0
        i32.load offset=200
        tee_local $p12
        i32.load offset=13076
        get_local $p8
        i32.lt_u
        tee_local $l6
        br_if $B5
        get_local $p12
        i32.load offset=13072
        get_local $p8
        i32.ge_u
        br_if $B5
        get_local $l1
        i32.const 31255
        i32.add
        i32.load8_u
        get_local $p9
        i32.le_s
        get_local $p9
        i32.eqz
        i32.const 0
        get_local $p11
        select
        i32.or
        br_if $B5
        get_local $p0
        i32.load offset=136
        tee_local $p11
        i32.const 224
        i32.add
        get_local $p11
        get_local $p8
        i32.sub
        i32.const 42
        i32.add
        call $get_cabac
        br $B4
      end
      get_local $l6
      block $B6 (result i32)
        i32.const 0
        get_local $p12
        i32.load offset=13088
        br_if $B6
        drop
        i32.const 0
        get_local $l1
        i32.const 31244
        i32.add
        i32.load
        br_if $B6
        drop
        get_local $p9
        i32.eqz
        get_local $l1
        i32.const 31248
        i32.add
        i32.load
        i32.const 0
        i32.ne
        i32.and
      end
      get_local $p9
      i32.eqz
      get_local $p11
      i32.const 0
      i32.ne
      i32.and
      i32.or
      i32.or
    end
    set_local $p12
    get_local $p0
    i32.load offset=200
    i32.load offset=4
    set_local $p11
    block $B7
      block $B8
        block $B9
          block $B10
            block $B11
              block $B12
                block $B13
                  get_local $p8
                  i32.const 2
                  i32.le_s
                  if $I14
                    get_local $p11
                    i32.const 3
                    i32.eq
                    br_if $B13
                    br $B12
                  end
                  get_local $p11
                  i32.eqz
                  br_if $B12
                end
                block $B15
                  get_local $p9
                  if $I16
                    i32.const 0
                    set_local $p11
                    get_local $l2
                    i32.eqz
                    br_if $B15
                  end
                  get_local $l0
                  get_local $p0
                  get_local $p9
                  call $ff_hevc_cbf_cb_cr_decode
                  tee_local $p11
                  i32.store offset=8
                  get_local $p0
                  i32.load offset=200
                  i32.load offset=4
                  i32.const 2
                  i32.ne
                  get_local $p8
                  i32.const 3
                  i32.ne
                  i32.const 0
                  get_local $p12
                  i32.const 255
                  i32.and
                  select
                  i32.or
                  br_if $B15
                  get_local $l0
                  get_local $p0
                  get_local $p9
                  call $ff_hevc_cbf_cb_cr_decode
                  tee_local $l4
                  i32.store offset=12
                end
                get_local $p9
                i32.eqz
                get_local $l3
                i32.or
                i32.eqz
                if $I17
                  i32.const 0
                  set_local $l3
                  br $B11
                end
                get_local $l0
                get_local $p0
                get_local $p9
                call $ff_hevc_cbf_cb_cr_decode
                tee_local $l3
                i32.store
                get_local $p0
                i32.load offset=200
                i32.load offset=4
                i32.const 2
                i32.ne
                br_if $B11
                get_local $p8
                i32.const 3
                i32.ne
                i32.const 0
                get_local $p12
                i32.const 255
                i32.and
                select
                br_if $B10
                get_local $l0
                get_local $p0
                get_local $p9
                call $ff_hevc_cbf_cb_cr_decode
                tee_local $l5
                i32.store offset=4
                br $B11
              end
              get_local $l2
              set_local $p11
            end
            get_local $p12
            i32.const 255
            i32.and
            i32.eqz
            br_if $B9
          end
          get_local $p0
          get_local $p1
          get_local $p2
          get_local $p1
          get_local $p2
          get_local $p5
          get_local $p6
          get_local $p7
          get_local $p8
          i32.const 1
          i32.sub
          tee_local $p3
          get_local $p9
          i32.const 1
          i32.add
          tee_local $p4
          i32.const 0
          get_local $l0
          i32.const 8
          i32.add
          get_local $l0
          call $hls_transform_tree
          tee_local $p9
          i32.const 0
          i32.lt_s
          br_if $B7
          get_local $p0
          i32.const 1
          get_local $p3
          i32.shl
          tee_local $p8
          get_local $p1
          i32.add
          tee_local $p10
          get_local $p2
          get_local $p1
          get_local $p2
          get_local $p5
          get_local $p6
          get_local $p7
          get_local $p3
          get_local $p4
          i32.const 1
          get_local $l0
          i32.const 8
          i32.add
          get_local $l0
          call $hls_transform_tree
          tee_local $p9
          i32.const 0
          i32.lt_s
          br_if $B7
          get_local $p0
          get_local $p1
          get_local $p2
          get_local $p8
          i32.add
          tee_local $p8
          get_local $p1
          get_local $p2
          get_local $p5
          get_local $p6
          get_local $p7
          get_local $p3
          get_local $p4
          i32.const 2
          get_local $l0
          i32.const 8
          i32.add
          get_local $l0
          call $hls_transform_tree
          tee_local $p9
          i32.const 0
          i32.lt_s
          br_if $B7
          get_local $p0
          get_local $p10
          get_local $p8
          get_local $p1
          get_local $p2
          get_local $p5
          get_local $p6
          get_local $p7
          get_local $p3
          get_local $p4
          i32.const 3
          get_local $l0
          i32.const 8
          i32.add
          get_local $l0
          call $hls_transform_tree
          tee_local $p9
          i32.const 0
          i32.ge_s
          br_if $B8
          br $B7
        end
        get_local $p0
        i32.load offset=200
        tee_local $p7
        i32.load offset=13072
        set_local $p12
        get_local $p7
        i32.load offset=13148
        set_local $l2
        get_local $p0
        get_local $p1
        get_local $p2
        get_local $p3
        get_local $p4
        get_local $p5
        get_local $p6
        get_local $p8
        get_local $p10
        block $B18 (result i32)
          get_local $p9
          get_local $p11
          i32.or
          get_local $l3
          i32.or
          get_local $l1
          i32.const 31244
          i32.add
          i32.load
          i32.const 1
          i32.eq
          i32.or
          i32.eqz
          if $I19
            i32.const 1
            get_local $l4
            get_local $l5
            i32.or
            i32.eqz
            get_local $p7
            i32.load offset=4
            i32.const 2
            i32.ne
            i32.or
            br_if $B18
            drop
          end
          get_local $p0
          i32.load offset=136
          tee_local $p3
          i32.const 224
          i32.add
          get_local $p3
          i32.const 40
          i32.const 41
          get_local $p9
          select
          i32.add
          call $get_cabac
        end
        tee_local $p3
        get_local $l0
        i32.const 8
        i32.add
        get_local $l0
        call $hls_transform_unit
        tee_local $p9
        i32.const 0
        i32.lt_s
        br_if $B7
        get_local $p3
        i32.eqz
        get_local $p8
        i32.const 31
        i32.eq
        i32.or
        i32.eqz
        if $I20
          i32.const 1
          get_local $p12
          i32.shl
          set_local $p3
          i32.const 1
          get_local $p8
          i32.shl
          set_local $p4
          i32.const 0
          set_local $p7
          loop $L21
            get_local $p2
            get_local $p7
            i32.add
            get_local $p12
            i32.shr_s
            get_local $l2
            i32.mul
            set_local $p5
            i32.const 0
            set_local $p9
            loop $L22
              get_local $p0
              i32.load offset=4344
              get_local $p1
              get_local $p9
              i32.add
              get_local $p12
              i32.shr_s
              get_local $p5
              i32.add
              i32.add
              i32.const 1
              i32.store8
              get_local $p3
              get_local $p9
              i32.add
              tee_local $p9
              get_local $p4
              i32.lt_s
              br_if $L22
            end
            get_local $p3
            get_local $p7
            i32.add
            tee_local $p7
            get_local $p4
            i32.lt_s
            br_if $L21
          end
        end
        get_local $p0
        i32.const 2061
        i32.add
        i32.load8_u
        br_if $B8
        get_local $p0
        get_local $p1
        get_local $p2
        get_local $p8
        call $ff_hevc_deblocking_boundary_strengths
        get_local $p0
        i32.load offset=204
        i32.load8_u offset=40
        i32.eqz
        br_if $B8
        get_local $l1
        i32.const 31256
        i32.add
        i32.load8_u
        i32.eqz
        br_if $B8
        get_local $p0
        get_local $p1
        get_local $p2
        get_local $p8
        call $set_deblocking_bypass
      end
      i32.const 0
      set_local $p9
    end
    get_local $l0
    i32.const 16
    i32.add
    set_global $g0
    get_local $p9)
  (func $skip_bytes.1 (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_local $p1
    get_local $p0
    i32.load offset=20
    get_local $p0
    i32.load offset=16
    tee_local $l0
    i32.const 1
    i32.sub
    get_local $l0
    get_local $p0
    i32.load
    tee_local $l1
    i32.const 1
    i32.and
    select
    tee_local $l0
    i32.const 1
    i32.sub
    get_local $l0
    get_local $l1
    i32.const 511
    i32.and
    select
    tee_local $l0
    i32.sub
    tee_local $l1
    i32.le_s
    if $I0 (result i32)
      get_local $p0
      get_local $p1
      get_local $l0
      i32.add
      get_local $l1
      get_local $p1
      i32.sub
      call $ff_init_cabac_decoder
      get_local $l0
    else
      get_local $l2
    end)
  (func $luma_intra_pred_mode (type $t13) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l0
    set_global $g0
    get_local $p2
    get_local $p0
    i32.load offset=200
    tee_local $l1
    i32.load offset=13084
    tee_local $l2
    i32.shr_s
    set_local $l4
    get_local $p1
    get_local $l2
    i32.shr_s
    set_local $l5
    get_local $p1
    i32.const -1
    get_local $l1
    i32.load offset=13080
    i32.shl
    tee_local $l8
    i32.const -1
    i32.xor
    tee_local $p1
    i32.and
    set_local $l9
    get_local $l1
    i32.load offset=13156
    set_local $l6
    block $B0 (result i32)
      get_local $p0
      i32.load offset=136
      tee_local $l7
      i32.load8_u offset=309
      i32.eqz
      if $I1
        i32.const 1
        get_local $p1
        get_local $p2
        i32.and
        i32.eqz
        br_if $B0
        drop
      end
      get_local $p0
      i32.load offset=4340
      get_local $l4
      i32.const 1
      i32.sub
      get_local $l6
      i32.mul
      get_local $l5
      i32.add
      i32.add
      i32.load8_u
    end
    set_local $l3
    get_local $p3
    get_local $l2
    i32.shr_s
    set_local $l2
    block $B2
      block $B3
        block $B4
          block $B5
            block $B6
              block $B7
                block $B8 (result i32)
                  get_local $l9
                  i32.eqz
                  if $I9
                    i32.const 1
                    get_local $l7
                    i32.load8_u offset=308
                    i32.eqz
                    br_if $B8
                    drop
                  end
                  get_local $p0
                  i32.load offset=4340
                  get_local $l5
                  get_local $l4
                  get_local $l6
                  i32.mul
                  i32.add
                  i32.add
                  i32.const 1
                  i32.sub
                  i32.load8_u
                end
                tee_local $p1
                get_local $l3
                i32.const 1
                get_local $p2
                get_local $l8
                i32.and
                get_local $p2
                i32.lt_s
                select
                tee_local $p2
                i32.eq
                if $I10
                  get_local $p1
                  i32.const 2
                  i32.lt_u
                  br_if $B6
                  get_local $l0
                  get_local $p1
                  i32.store offset=4
                  get_local $l0
                  get_local $p1
                  i32.const 1
                  i32.sub
                  i32.const 31
                  i32.and
                  i32.const 2
                  i32.add
                  tee_local $l1
                  i32.store offset=12
                  get_local $l0
                  get_local $p1
                  i32.const 29
                  i32.add
                  i32.const 31
                  i32.and
                  i32.const 2
                  i32.add
                  tee_local $p2
                  i32.store offset=8
                  br $B7
                end
                get_local $l0
                get_local $p2
                i32.store offset=8
                get_local $l0
                get_local $p1
                i32.store offset=4
                get_local $l0
                block $B11 (result i32)
                  i32.const 0
                  get_local $p2
                  i32.eqz
                  get_local $p1
                  i32.eqz
                  i32.or
                  i32.eqz
                  br_if $B11
                  drop
                  i32.const 1
                  get_local $p2
                  i32.const 1
                  i32.eq
                  get_local $p1
                  i32.const 1
                  i32.eq
                  i32.or
                  i32.eqz
                  br_if $B11
                  drop
                  i32.const 26
                end
                tee_local $l1
                i32.store offset=12
              end
              get_local $p4
              i32.eqz
              br_if $B5
              br $B3
            end
            get_local $l0
            i32.const 26
            i32.store offset=12
            get_local $l0
            i64.const 4294967296
            i64.store offset=4 align=4
            get_local $p4
            br_if $B3
            i32.const 0
            set_local $l3
            br $B4
          end
          block $B12
            get_local $p1
            get_local $p2
            i32.le_s
            if $I13
              get_local $p2
              set_local $p3
              get_local $p1
              set_local $p2
              br $B12
            end
            get_local $l0
            get_local $p1
            i32.store offset=8
            get_local $p1
            set_local $p3
          end
          block $B14
            get_local $p2
            get_local $l1
            i32.le_s
            if $I15
              get_local $p2
              set_local $l3
              get_local $l1
              set_local $p2
              br $B14
            end
            get_local $l0
            get_local $p2
            i32.store offset=12
            get_local $l1
            set_local $l3
          end
          get_local $p2
          get_local $p3
          i32.ge_s
          br_if $B4
          get_local $l0
          get_local $p2
          i32.store offset=8
          get_local $l0
          get_local $p3
          i32.store offset=12
        end
        get_local $l7
        i32.const 31264
        i32.add
        i32.load
        tee_local $p1
        get_local $p1
        get_local $l3
        i32.ge_s
        i32.add
        set_local $p2
        i32.const 1
        set_local $p1
        loop $L16
          get_local $p2
          get_local $p2
          get_local $l0
          i32.const 4
          i32.add
          get_local $p1
          i32.const 2
          i32.shl
          i32.add
          i32.load
          i32.ge_s
          i32.add
          set_local $p2
          get_local $p1
          i32.const 1
          i32.add
          tee_local $p1
          i32.const 3
          i32.ne
          br_if $L16
        end
        br $B2
      end
      get_local $l0
      i32.const 4
      i32.add
      get_local $l7
      i32.load offset=31260
      i32.const 2
      i32.shl
      i32.add
      i32.load
      set_local $p2
    end
    get_local $l2
    i32.const 1
    get_local $l2
    select
    tee_local $p3
    i32.const 1
    i32.ge_s
    if $I17
      i32.const 0
      set_local $p1
      loop $L18
        get_local $p0
        i32.load offset=4340
        get_local $p1
        get_local $l4
        i32.add
        get_local $l6
        i32.mul
        get_local $l5
        i32.add
        i32.add
        get_local $p2
        get_local $p3
        call $memset
        drop
        get_local $p1
        i32.const 1
        i32.add
        tee_local $p1
        get_local $p3
        i32.ne
        br_if $L18
      end
    end
    get_local $l0
    i32.const 16
    i32.add
    set_global $g0
    get_local $p2)
  (func $hls_transform_unit (type $t22) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (param $p8 i32) (param $p9 i32) (param $p10 i32) (param $p11 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32)
    get_local $p0
    i32.load offset=200
    i32.const 13172
    i32.add
    i32.load
    set_local $l2
    get_local $p0
    i32.load offset=136
    tee_local $l0
    i32.const 31244
    i32.add
    i32.load
    i32.const 1
    i32.eq
    if $I0
      get_local $p0
      get_local $p1
      get_local $p2
      i32.const 1
      get_local $p7
      i32.shl
      tee_local $l1
      get_local $l1
      call $ff_hevc_set_neighbour_available
      get_local $p0
      get_local $p1
      get_local $p2
      get_local $p7
      i32.const 0
      call $intra_pred
    end
    get_local $p7
    get_local $l2
    i32.sub
    set_local $l2
    get_local $p10
    i32.load
    set_local $l3
    block $B1
      block $B2
        block $B3
          block $B4
            block $B5
              block $B6
                get_local $p9
                i32.eqz
                if $I7
                  i32.const 1
                  set_local $l1
                  get_local $l3
                  br_if $B5
                  get_local $p11
                  i32.load
                  br_if $B5
                  get_local $p0
                  i32.load offset=200
                  tee_local $l1
                  i32.load offset=4
                  tee_local $l4
                  i32.const 2
                  i32.ne
                  br_if $B4
                  get_local $p10
                  i32.load offset=4
                  br_if $B6
                  get_local $p11
                  i32.load offset=4
                  br_if $B6
                  i32.const 0
                  set_local $l3
                  get_local $l0
                  i32.load offset=31244
                  i32.const 1
                  i32.eq
                  br_if $B3
                  br $B2
                end
                i32.const 1
                set_local $l1
                get_local $l3
                br_if $B5
              end
              i32.const 1
              set_local $l1
              get_local $p11
              i32.load
              br_if $B5
              i32.const 0
              set_local $l1
              get_local $p0
              i32.load offset=200
              i32.load offset=4
              i32.const 2
              i32.ne
              br_if $B5
              i32.const 1
              set_local $l1
              get_local $p10
              i32.load offset=4
              br_if $B5
              get_local $p11
              i32.load offset=4
              i32.const 0
              i32.ne
              set_local $l1
            end
            block $B8
              get_local $p0
              i32.load offset=204
              i32.load8_u offset=22
              i32.eqz
              br_if $B8
              get_local $l0
              i32.load8_u offset=300
              br_if $B8
              get_local $l0
              get_local $p0
              call $ff_hevc_cu_qp_delta_abs
              tee_local $l3
              i32.store offset=280
              block $B9
                get_local $l3
                i32.eqz
                if $I10
                  i32.const 0
                  set_local $l4
                  br $B9
                end
                get_local $p0
                call $ff_hevc_sao_offset_sign_decode
                set_local $l3
                get_local $l0
                i32.load offset=280
                set_local $l4
                get_local $l3
                i32.const 1
                i32.ne
                br_if $B9
                get_local $l0
                i32.const 0
                get_local $l4
                i32.sub
                tee_local $l4
                i32.store offset=280
              end
              get_local $l0
              i32.const 1
              i32.store8 offset=300
              i32.const -1094995529
              set_local $l3
              get_local $l4
              i32.const -26
              get_local $p0
              i32.load offset=200
              i32.load offset=13192
              i32.const 2
              i32.div_s
              tee_local $l5
              i32.sub
              i32.lt_s
              get_local $l4
              get_local $l5
              i32.const 25
              i32.add
              i32.gt_s
              i32.or
              br_if $B2
              get_local $p0
              get_local $p5
              get_local $p6
              call $ff_hevc_set_qPy
            end
            block $B11
              get_local $p0
              i32.const 2080
              i32.add
              i32.load8_u
              i32.eqz
              get_local $l1
              i32.const 1
              i32.xor
              i32.or
              br_if $B11
              get_local $l0
              i32.const 31256
              i32.add
              i32.load8_u
              br_if $B11
              get_local $l0
              i32.load8_u offset=301
              br_if $B11
              block $B12 (result i32)
                get_local $p0
                i32.load offset=136
                tee_local $p5
                i32.const 224
                i32.add
                get_local $p5
                i32.const 176
                i32.add
                call $get_cabac
                if $I13
                  block $B14
                    get_local $p0
                    i32.load offset=204
                    tee_local $l1
                    i32.load8_u offset=1633
                    i32.eqz
                    if $I15
                      i32.const 0
                      set_local $l3
                      br $B14
                    end
                    get_local $p0
                    call $ff_hevc_cu_chroma_qp_offset_idx
                    set_local $l3
                    get_local $p0
                    i32.load offset=204
                    set_local $l1
                  end
                  get_local $l0
                  get_local $l1
                  get_local $l3
                  i32.add
                  tee_local $p5
                  i32.const 1634
                  i32.add
                  i32.load8_u
                  i32.store8 offset=302
                  get_local $p5
                  i32.const 1639
                  i32.add
                  i32.load8_u
                  br $B12
                end
                get_local $l0
                i32.const 0
                i32.store8 offset=302
                i32.const 0
              end
              set_local $p5
              get_local $l0
              i32.const 1
              i32.store8 offset=301
              get_local $l0
              get_local $p5
              i32.store8 offset=303
            end
            i32.const 0
            set_local $l1
            block $B16 (result i32)
              i32.const 0
              get_local $p7
              i32.const 3
              i32.gt_s
              br_if $B16
              drop
              i32.const 0
              get_local $l0
              i32.load offset=31244
              i32.const 1
              i32.ne
              br_if $B16
              drop
              i32.const 2
              get_local $l0
              i32.load offset=288
              tee_local $p5
              i32.const 22
              i32.sub
              i32.const 9
              i32.lt_u
              get_local $p5
              i32.const 6
              i32.sub
              i32.const 9
              i32.lt_u
              select
              set_local $l1
              i32.const 2
              get_local $l0
              i32.load offset=292
              tee_local $p5
              i32.const 6
              i32.sub
              i32.const 9
              i32.lt_u
              br_if $B16
              drop
              get_local $p5
              i32.const 22
              i32.sub
              i32.const 9
              i32.lt_u
            end
            set_local $p5
            get_local $l0
            i32.const 0
            i32.store8 offset=304
            get_local $p9
            if $I17
              get_local $p0
              get_local $p1
              get_local $p2
              get_local $p7
              get_local $l1
              i32.const 0
              call $ff_hevc_hls_residual_coding
            end
            i32.const 0
            set_local $l3
            get_local $p0
            i32.load offset=200
            tee_local $p6
            i32.load offset=4
            tee_local $l1
            i32.eqz
            br_if $B2
            get_local $p7
            i32.const 2
            i32.le_s
            i32.const 0
            get_local $l1
            i32.const 3
            i32.ne
            select
            i32.eqz
            if $I18
              get_local $p6
              i32.const 13184
              i32.add
              i32.load
              get_local $l2
              i32.add
              set_local $p3
              get_local $p6
              i32.const 13172
              i32.add
              i32.load
              get_local $l2
              i32.add
              set_local $p4
              block $B19
                block $B20
                  get_local $p9
                  if $I21
                    get_local $p0
                    i32.load offset=204
                    i32.load8_u offset=1630
                    br_if $B20
                  end
                  get_local $l0
                  i32.const 0
                  i32.store8 offset=304
                  br $B19
                end
                block $B22
                  get_local $l0
                  i32.load offset=31244
                  i32.eqz
                  if $I23
                    get_local $l0
                    i32.const 1
                    i32.store8 offset=304
                    br $B22
                  end
                  get_local $l0
                  get_local $l0
                  i32.load offset=296
                  tee_local $p6
                  i32.const 4
                  i32.eq
                  i32.store8 offset=304
                  get_local $p6
                  i32.const 4
                  i32.ne
                  br_if $B19
                end
                get_local $p0
                i32.const 0
                call $hls_cross_component_pred
              end
              i32.const 1
              get_local $p3
              i32.shl
              set_local $p6
              i32.const 1
              get_local $p4
              i32.shl
              set_local $p8
              get_local $l0
              i32.const 11680
              i32.add
              set_local $p4
              get_local $l0
              i32.const 320
              i32.add
              set_local $p9
              i32.const 1
              get_local $l2
              i32.shl
              get_local $l2
              i32.shl
              set_local $p3
              get_local $l2
              i32.const 2
              i32.shl
              get_local $p0
              i32.add
              i32.const 2604
              i32.add
              set_local $l1
              i32.const 0
              set_local $p7
              loop $L24
                get_local $l0
                i32.load offset=31244
                i32.const 1
                i32.eq
                if $I25
                  get_local $p0
                  get_local $p1
                  get_local $p7
                  get_local $l2
                  i32.shl
                  get_local $p2
                  i32.add
                  tee_local $l3
                  get_local $p8
                  get_local $p6
                  call $ff_hevc_set_neighbour_available
                  get_local $p0
                  get_local $p1
                  get_local $l3
                  get_local $l2
                  i32.const 1
                  call $intra_pred
                end
                block $B26
                  get_local $p10
                  get_local $p7
                  i32.const 2
                  i32.shl
                  i32.add
                  i32.load
                  if $I27
                    get_local $p0
                    get_local $p1
                    get_local $p7
                    get_local $l2
                    i32.shl
                    get_local $p2
                    i32.add
                    get_local $l2
                    get_local $p5
                    i32.const 1
                    call $ff_hevc_hls_residual_coding
                    br $B26
                  end
                  get_local $l0
                  i32.load8_u offset=304
                  i32.eqz
                  br_if $B26
                  get_local $p0
                  i32.load offset=160
                  tee_local $l3
                  i32.load offset=4
                  get_local $p1
                  get_local $p0
                  i32.load offset=200
                  tee_local $p7
                  i32.const 13172
                  i32.add
                  i32.load
                  i32.shr_s
                  get_local $p7
                  i32.load offset=56
                  i32.shl
                  get_local $l3
                  i32.load offset=36
                  tee_local $l3
                  get_local $p2
                  get_local $p7
                  i32.const 13184
                  i32.add
                  i32.load
                  i32.shr_s
                  i32.mul
                  i32.add
                  i32.add
                  set_local $l4
                  get_local $p3
                  i32.const 1
                  i32.lt_s
                  if $I28 (result i32)
                    i32.const 0
                  else
                    get_local $l0
                    i32.load offset=284
                    set_local $l5
                    i32.const 0
                    set_local $p7
                    loop $L29
                      get_local $p4
                      get_local $p7
                      i32.const 1
                      i32.shl
                      tee_local $l6
                      i32.add
                      get_local $l5
                      get_local $p9
                      get_local $l6
                      i32.add
                      i32.load16_s
                      i32.mul
                      i32.const 3
                      i32.shr_u
                      i32.store16
                      get_local $p7
                      i32.const 1
                      i32.add
                      tee_local $p7
                      get_local $p3
                      i32.ne
                      br_if $L29
                    end
                    get_local $p3
                  end
                  set_local $p7
                  get_local $l4
                  get_local $p4
                  get_local $l3
                  get_local $l1
                  i32.load
                  call_indirect (type $t5)
                end
                get_local $p7
                i32.const 1
                i32.add
                tee_local $p7
                i32.const 2
                i32.const 1
                get_local $p0
                i32.load offset=200
                i32.load offset=4
                i32.const 2
                i32.eq
                select
                i32.lt_s
                br_if $L24
              end
              get_local $l0
              i32.load8_u offset=304
              if $I30
                get_local $p0
                i32.const 1
                call $hls_cross_component_pred
              end
              i32.const 0
              set_local $p7
              loop $L31
                get_local $l0
                i32.load offset=31244
                i32.const 1
                i32.eq
                if $I32
                  get_local $p0
                  get_local $p1
                  get_local $p7
                  get_local $l2
                  i32.shl
                  get_local $p2
                  i32.add
                  tee_local $p10
                  get_local $p8
                  get_local $p6
                  call $ff_hevc_set_neighbour_available
                  get_local $p0
                  get_local $p1
                  get_local $p10
                  get_local $l2
                  i32.const 2
                  call $intra_pred
                end
                block $B33
                  get_local $p11
                  get_local $p7
                  i32.const 2
                  i32.shl
                  i32.add
                  i32.load
                  if $I34
                    get_local $p0
                    get_local $p1
                    get_local $p7
                    get_local $l2
                    i32.shl
                    get_local $p2
                    i32.add
                    get_local $l2
                    get_local $p5
                    i32.const 2
                    call $ff_hevc_hls_residual_coding
                    br $B33
                  end
                  get_local $l0
                  i32.load8_u offset=304
                  i32.eqz
                  br_if $B33
                  get_local $p0
                  i32.load offset=160
                  tee_local $p10
                  i32.load offset=8
                  get_local $p1
                  get_local $p0
                  i32.load offset=200
                  tee_local $p7
                  i32.const 13176
                  i32.add
                  i32.load
                  i32.shr_s
                  get_local $p7
                  i32.load offset=56
                  i32.shl
                  get_local $p10
                  i32.load offset=40
                  tee_local $p10
                  get_local $p2
                  get_local $p7
                  i32.const 13188
                  i32.add
                  i32.load
                  i32.shr_s
                  i32.mul
                  i32.add
                  i32.add
                  set_local $l3
                  get_local $p3
                  i32.const 1
                  i32.lt_s
                  if $I35 (result i32)
                    i32.const 0
                  else
                    get_local $l0
                    i32.load offset=284
                    set_local $l4
                    i32.const 0
                    set_local $p7
                    loop $L36
                      get_local $p4
                      get_local $p7
                      i32.const 1
                      i32.shl
                      tee_local $l5
                      i32.add
                      get_local $l4
                      get_local $p9
                      get_local $l5
                      i32.add
                      i32.load16_s
                      i32.mul
                      i32.const 3
                      i32.shr_u
                      i32.store16
                      get_local $p7
                      i32.const 1
                      i32.add
                      tee_local $p7
                      get_local $p3
                      i32.ne
                      br_if $L36
                    end
                    get_local $p3
                  end
                  set_local $p7
                  get_local $l3
                  get_local $p4
                  get_local $p10
                  get_local $l1
                  i32.load
                  call_indirect (type $t5)
                end
                get_local $p7
                i32.const 1
                i32.add
                tee_local $p7
                i32.const 2
                i32.const 1
                get_local $p0
                i32.load offset=200
                i32.load offset=4
                i32.const 2
                i32.eq
                select
                i32.lt_s
                br_if $L31
              end
              br $B1
            end
            get_local $p8
            i32.const 3
            i32.ne
            br_if $B2
            i32.const 1
            get_local $p7
            i32.const 1
            i32.add
            i32.shl
            set_local $p1
            i32.const 1
            get_local $p6
            i32.const 13184
            i32.add
            i32.load
            get_local $p7
            i32.add
            i32.shl
            set_local $p2
            i32.const 0
            set_local $p9
            loop $L37
              get_local $l0
              i32.load offset=31244
              i32.const 1
              i32.eq
              if $I38
                get_local $p0
                get_local $p3
                get_local $p9
                get_local $p7
                i32.shl
                get_local $p4
                i32.add
                tee_local $p6
                get_local $p1
                get_local $p2
                call $ff_hevc_set_neighbour_available
                get_local $p0
                get_local $p3
                get_local $p6
                get_local $p7
                i32.const 1
                call $intra_pred
              end
              get_local $p10
              get_local $p9
              i32.const 2
              i32.shl
              i32.add
              i32.load
              if $I39
                get_local $p0
                get_local $p3
                get_local $p9
                get_local $p7
                i32.shl
                get_local $p4
                i32.add
                get_local $p7
                get_local $p5
                i32.const 1
                call $ff_hevc_hls_residual_coding
              end
              get_local $p9
              i32.const 1
              i32.add
              tee_local $p9
              i32.const 2
              i32.const 1
              get_local $p0
              i32.load offset=200
              i32.load offset=4
              i32.const 2
              i32.eq
              select
              i32.lt_u
              br_if $L37
            end
            i32.const 0
            set_local $p9
            loop $L40
              get_local $l0
              i32.load offset=31244
              i32.const 1
              i32.eq
              if $I41
                get_local $p0
                get_local $p3
                get_local $p9
                get_local $p7
                i32.shl
                get_local $p4
                i32.add
                tee_local $p6
                get_local $p1
                get_local $p2
                call $ff_hevc_set_neighbour_available
                get_local $p0
                get_local $p3
                get_local $p6
                get_local $p7
                i32.const 2
                call $intra_pred
              end
              get_local $p11
              get_local $p9
              i32.const 2
              i32.shl
              i32.add
              i32.load
              if $I42
                get_local $p0
                get_local $p3
                get_local $p9
                get_local $p7
                i32.shl
                get_local $p4
                i32.add
                get_local $p7
                get_local $p5
                i32.const 2
                call $ff_hevc_hls_residual_coding
              end
              get_local $p9
              i32.const 1
              i32.add
              tee_local $p9
              i32.const 2
              i32.const 1
              get_local $p0
              i32.load offset=200
              i32.load offset=4
              i32.const 2
              i32.eq
              select
              i32.lt_u
              br_if $L40
            end
            br $B1
          end
          i32.const 0
          set_local $l3
          get_local $l4
          i32.eqz
          br_if $B2
          get_local $l0
          i32.load offset=31244
          i32.const 1
          i32.ne
          br_if $B2
        end
        get_local $p7
        i32.const 2
        i32.le_s
        i32.const 0
        get_local $l4
        i32.const 3
        i32.ne
        select
        i32.eqz
        if $I43
          get_local $p0
          get_local $p1
          get_local $p2
          i32.const 1
          get_local $l1
          i32.const 13172
          i32.add
          i32.load
          get_local $l2
          i32.add
          i32.shl
          tee_local $p3
          i32.const 1
          get_local $l1
          i32.const 13184
          i32.add
          i32.load
          get_local $l2
          i32.add
          i32.shl
          tee_local $p4
          call $ff_hevc_set_neighbour_available
          get_local $p0
          get_local $p1
          get_local $p2
          get_local $l2
          i32.const 1
          call $intra_pred
          get_local $p0
          get_local $p1
          get_local $p2
          get_local $l2
          i32.const 2
          call $intra_pred
          get_local $p0
          i32.load offset=200
          i32.load offset=4
          i32.const 2
          i32.ne
          br_if $B2
          get_local $p0
          get_local $p1
          i32.const 1
          get_local $l2
          i32.shl
          get_local $p2
          i32.add
          tee_local $p2
          get_local $p3
          get_local $p4
          call $ff_hevc_set_neighbour_available
          get_local $p0
          get_local $p1
          get_local $p2
          get_local $l2
          i32.const 1
          call $intra_pred
          get_local $p0
          get_local $p1
          get_local $p2
          get_local $l2
          i32.const 2
          call $intra_pred
          i32.const 0
          return
        end
        get_local $p8
        i32.const 3
        i32.ne
        br_if $B2
        get_local $p0
        get_local $p3
        get_local $p4
        i32.const 1
        get_local $p7
        i32.const 1
        i32.add
        i32.shl
        tee_local $p2
        i32.const 1
        get_local $l1
        i32.const 13184
        i32.add
        i32.load
        get_local $p7
        i32.add
        i32.shl
        tee_local $p5
        call $ff_hevc_set_neighbour_available
        get_local $p0
        get_local $p3
        get_local $p4
        get_local $p7
        i32.const 1
        call $intra_pred
        get_local $p0
        get_local $p3
        get_local $p4
        get_local $p7
        i32.const 2
        call $intra_pred
        get_local $p0
        i32.load offset=200
        i32.load offset=4
        i32.const 2
        i32.ne
        br_if $B2
        get_local $p0
        get_local $p3
        i32.const 1
        get_local $p7
        i32.shl
        get_local $p4
        i32.add
        tee_local $p1
        get_local $p2
        get_local $p5
        call $ff_hevc_set_neighbour_available
        get_local $p0
        get_local $p3
        get_local $p1
        get_local $p7
        i32.const 1
        call $intra_pred
        get_local $p0
        get_local $p3
        get_local $p1
        get_local $p7
        i32.const 2
        call $intra_pred
      end
      get_local $l3
      return
    end
    i32.const 0)
  (func $hls_cross_component_pred (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32)
    get_local $p0
    i32.load offset=136
    block $B0 (result i32)
      i32.const 0
      get_local $p0
      get_local $p1
      call $ff_hevc_log2_res_scale_abs
      tee_local $l0
      i32.eqz
      br_if $B0
      drop
      i32.const 1
      get_local $p0
      i32.load offset=136
      tee_local $p0
      i32.const 224
      i32.add
      get_local $p0
      get_local $p1
      i32.add
      i32.const 174
      i32.add
      call $get_cabac
      i32.const 1
      i32.shl
      i32.sub
      get_local $l0
      i32.const 1
      i32.sub
      i32.shl
    end
    i32.store offset=284)
  (func $intra_pred (type $t8) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32) (local $l13 i32) (local $l14 i32) (local $l15 i32) (local $l16 i32) (local $l17 i32) (local $l18 i32) (local $l19 i32) (local $l20 i32) (local $l21 i32) (local $l22 i32) (local $l23 i32) (local $l24 i32) (local $l25 i32) (local $l26 i32)
    get_global $g0
    i32.const 320
    i32.sub
    tee_local $l0
    set_global $g0
    get_local $p4
    i32.const 2
    i32.shl
    tee_local $l2
    get_local $p0
    i32.load offset=160
    i32.add
    tee_local $l7
    set_local $l12
    get_local $p0
    i32.load offset=200
    tee_local $l10
    get_local $l2
    i32.add
    tee_local $l2
    i32.const 13168
    i32.add
    i32.load
    set_local $l14
    i32.const 1
    get_local $p3
    i32.shl
    tee_local $l1
    get_local $l2
    i32.const 13180
    i32.add
    i32.load
    tee_local $l18
    i32.shl
    set_local $l11
    get_local $p0
    i32.load offset=204
    tee_local $l22
    i32.load offset=1684
    tee_local $l4
    get_local $l10
    i32.load offset=13164
    tee_local $l2
    get_local $p2
    get_local $l10
    i32.load offset=13072
    tee_local $l5
    i32.shr_s
    i32.and
    tee_local $l8
    get_local $l2
    i32.const 2
    i32.add
    tee_local $l3
    i32.mul
    get_local $p1
    get_local $l5
    i32.shr_s
    get_local $l2
    i32.and
    tee_local $l13
    i32.add
    i32.const 2
    i32.shl
    i32.add
    i32.load
    set_local $l15
    get_local $p0
    i32.load offset=136
    tee_local $l17
    i32.load offset=31288
    if $I0
      get_local $l15
      get_local $l13
      get_local $l8
      get_local $l11
      get_local $l5
      i32.shr_s
      i32.add
      get_local $l2
      i32.and
      get_local $l3
      i32.mul
      i32.add
      i32.const 2
      i32.shl
      get_local $l4
      i32.add
      i32.const 4
      i32.sub
      i32.load
      i32.gt_s
      set_local $l6
    end
    get_local $l12
    i32.load offset=32
    set_local $l12
    get_local $l7
    i32.load
    set_local $l25
    get_local $l1
    get_local $l14
    i32.shl
    set_local $l7
    get_local $l17
    i32.const 31304
    i32.add
    i32.load
    if $I1
      get_local $l15
      get_local $l4
      get_local $l8
      i32.const 1
      i32.sub
      get_local $l3
      i32.mul
      get_local $l13
      get_local $l7
      get_local $l5
      i32.shr_s
      i32.add
      get_local $l2
      i32.and
      i32.add
      i32.const 2
      i32.shl
      i32.add
      i32.load
      i32.gt_s
      set_local $l9
    end
    i32.const 292
    i32.const 288
    get_local $p4
    select
    get_local $l17
    i32.add
    i32.load
    set_local $l13
    get_local $l0
    i32.const 240
    i32.add
    i32.const 1
    i32.or
    set_local $l5
    get_local $l17
    i32.const 31296
    i32.add
    i32.load
    set_local $l4
    get_local $l17
    i32.const 31300
    i32.add
    i32.load
    set_local $l15
    get_local $l17
    i32.const 31292
    i32.add
    i32.load
    set_local $l8
    get_local $l10
    i32.load offset=13120
    set_local $l19
    get_local $p1
    get_local $l7
    i32.add
    set_local $l23
    get_local $l10
    i32.load offset=13124
    set_local $l24
    get_local $p2
    get_local $l11
    i32.add
    set_local $l20
    get_local $l22
    i32.load8_u offset=20
    i32.const 1
    i32.eq
    if $I2
      i32.const 0
      set_local $p0
      get_local $l11
      get_local $l10
      i32.load offset=13084
      tee_local $l2
      i32.shr_s
      set_local $l3
      block $B3
        i32.const -1
        get_local $l2
        i32.shl
        i32.const -1
        i32.xor
        tee_local $l21
        get_local $p1
        i32.and
        tee_local $l26
        get_local $l6
        i32.const 1
        i32.xor
        i32.or
        br_if $B3
        get_local $l10
        i32.load offset=13160
        get_local $l20
        get_local $l2
        i32.shr_s
        i32.sub
        tee_local $l6
        get_local $l3
        get_local $l3
        get_local $l6
        i32.gt_s
        select
        tee_local $l16
        i32.const 1
        i32.lt_s
        if $I4
          i32.const 0
          set_local $l6
          br $B3
        end
        i32.const 0
        set_local $l6
        loop $L5
          get_local $l6
          i32.const 1
          i32.or
          set_local $l6
          get_local $p0
          i32.const 2
          i32.add
          tee_local $p0
          get_local $l16
          i32.lt_s
          br_if $L5
        end
      end
      get_local $l7
      get_local $l2
      i32.shr_s
      set_local $l16
      block $B6
        get_local $l8
        i32.const 1
        i32.ne
        get_local $l26
        i32.or
        br_if $B6
        i32.const 0
        set_local $p0
        get_local $l10
        i32.load offset=13160
        get_local $p2
        get_local $l2
        i32.shr_s
        i32.sub
        tee_local $l8
        get_local $l3
        get_local $l3
        get_local $l8
        i32.gt_s
        select
        tee_local $l3
        i32.const 1
        i32.lt_s
        if $I7
          i32.const 0
          set_local $l8
          br $B6
        end
        i32.const 0
        set_local $l8
        loop $L8
          get_local $l8
          i32.const 1
          i32.or
          set_local $l8
          get_local $p0
          i32.const 2
          i32.add
          tee_local $p0
          get_local $l3
          i32.lt_s
          br_if $L8
        end
      end
      get_local $l16
      i32.const 1
      get_local $l16
      select
      set_local $l3
      block $B9
        get_local $p2
        get_local $l21
        i32.and
        tee_local $l16
        get_local $l4
        i32.const 1
        i32.ne
        i32.or
        br_if $B9
        i32.const 0
        set_local $p0
        get_local $l10
        i32.load offset=13156
        get_local $p1
        get_local $l2
        i32.shr_s
        i32.sub
        tee_local $l4
        get_local $l3
        get_local $l3
        get_local $l4
        i32.gt_s
        select
        tee_local $l21
        i32.const 1
        i32.lt_s
        if $I10
          i32.const 0
          set_local $l4
          br $B9
        end
        i32.const 0
        set_local $l4
        loop $L11
          get_local $l4
          i32.const 1
          i32.or
          set_local $l4
          get_local $p0
          i32.const 2
          i32.add
          tee_local $p0
          get_local $l21
          i32.lt_s
          br_if $L11
        end
      end
      i32.const 0
      set_local $p0
      block $B12
        get_local $l16
        get_local $l9
        i32.const 1
        i32.xor
        i32.or
        br_if $B12
        get_local $l10
        i32.load offset=13156
        get_local $l23
        get_local $l2
        i32.shr_s
        i32.sub
        tee_local $l2
        get_local $l3
        get_local $l2
        get_local $l3
        i32.lt_s
        select
        tee_local $l2
        i32.const 1
        i32.lt_s
        if $I13
          i32.const 0
          set_local $l9
          br $B12
        end
        i32.const 0
        set_local $l9
        loop $L14
          get_local $l9
          i32.const 1
          i32.or
          set_local $l9
          get_local $p0
          i32.const 2
          i32.add
          tee_local $p0
          get_local $l2
          i32.lt_s
          br_if $L14
        end
      end
      get_local $l5
      i32.const 128
      i32.const 64
      call $memset
      drop
      get_local $l0
      i32.const 80
      i32.add
      i32.const 128
      i32.const 65
      call $memset
      drop
    end
    get_local $l25
    get_local $p1
    get_local $l14
    i32.shr_s
    i32.add
    get_local $l12
    get_local $p2
    get_local $l18
    i32.shr_s
    i32.mul
    i32.add
    set_local $l3
    get_local $l15
    if $I15
      get_local $l0
      get_local $l3
      get_local $l12
      i32.const -1
      i32.xor
      i32.add
      i32.load8_u
      tee_local $p0
      i32.store8 offset=80
      get_local $l0
      get_local $p0
      i32.store8 offset=240
    end
    get_local $l0
    i32.const 80
    i32.add
    i32.const 1
    i32.or
    set_local $l2
    get_local $l4
    if $I16
      get_local $l2
      get_local $l3
      get_local $l12
      i32.sub
      get_local $l1
      call $memcpy
      drop
    end
    block $B17
      get_local $l9
      i32.eqz
      br_if $B17
      get_local $l1
      get_local $l2
      i32.add
      get_local $l3
      get_local $l12
      i32.sub
      get_local $l1
      i32.add
      get_local $l1
      call $memcpy
      set_local $l16
      get_local $l1
      get_local $l19
      get_local $l7
      i32.const 1
      i32.shl
      get_local $p1
      i32.add
      tee_local $p0
      get_local $p0
      get_local $l19
      i32.gt_s
      select
      get_local $l23
      i32.sub
      get_local $l14
      i32.shr_s
      tee_local $p0
      i32.sub
      tee_local $l7
      i32.const 1
      i32.lt_s
      br_if $B17
      get_local $l3
      get_local $l1
      get_local $l12
      i32.const -1
      i32.xor
      i32.add
      get_local $p0
      i32.add
      i32.add
      i32.load8_u
      i32.const 16843009
      i32.mul
      set_local $l14
      get_local $p0
      get_local $l16
      i32.add
      set_local $l19
      i32.const 0
      set_local $p0
      loop $L18
        get_local $p0
        get_local $l19
        i32.add
        get_local $l14
        i32.store align=1
        get_local $p0
        i32.const 4
        i32.add
        tee_local $p0
        get_local $l7
        i32.lt_s
        br_if $L18
      end
    end
    get_local $l8
    i32.eqz
    get_local $p3
    i32.const 31
    i32.eq
    i32.or
    i32.eqz
    if $I19
      get_local $l1
      i32.const 1
      get_local $l1
      i32.const 1
      i32.gt_s
      select
      set_local $l7
      i32.const 0
      set_local $p0
      loop $L20
        get_local $p0
        get_local $l5
        i32.add
        get_local $p0
        get_local $l12
        i32.mul
        get_local $l3
        i32.add
        i32.const 1
        i32.sub
        i32.load8_u
        i32.store8
        get_local $p0
        i32.const 1
        i32.add
        tee_local $p0
        get_local $l7
        i32.ne
        br_if $L20
      end
    end
    block $B21
      get_local $l6
      i32.eqz
      br_if $B21
      get_local $l24
      get_local $l11
      i32.const 1
      i32.shl
      get_local $p2
      i32.add
      tee_local $p0
      get_local $p0
      get_local $l24
      i32.gt_s
      select
      get_local $l20
      i32.sub
      get_local $l18
      i32.shr_s
      tee_local $l11
      get_local $l1
      i32.add
      set_local $l7
      get_local $l11
      i32.const 1
      i32.ge_s
      if $I22
        get_local $l1
        set_local $p0
        loop $L23
          get_local $p0
          get_local $l5
          i32.add
          get_local $p0
          get_local $l12
          i32.mul
          get_local $l3
          i32.add
          i32.const 1
          i32.sub
          i32.load8_u
          i32.store8
          get_local $p0
          i32.const 1
          i32.add
          tee_local $p0
          get_local $l7
          i32.lt_s
          br_if $L23
        end
      end
      get_local $l1
      get_local $l11
      i32.sub
      tee_local $l14
      i32.const 1
      i32.lt_s
      br_if $B21
      get_local $l7
      i32.const 1
      i32.sub
      get_local $l12
      i32.mul
      get_local $l3
      i32.add
      i32.const 1
      i32.sub
      i32.load8_u
      i32.const 16843009
      i32.mul
      set_local $l7
      get_local $l1
      get_local $l5
      i32.add
      get_local $l11
      i32.add
      set_local $l11
      i32.const 0
      set_local $p0
      loop $L24
        get_local $p0
        get_local $l11
        i32.add
        get_local $l7
        i32.store align=1
        get_local $p0
        i32.const 4
        i32.add
        tee_local $p0
        get_local $l14
        i32.lt_s
        br_if $L24
      end
    end
    get_local $l4
    get_local $l9
    i32.or
    get_local $l6
    get_local $l8
    i32.or
    get_local $l15
    i32.or
    i32.or
    i32.eqz
    get_local $l22
    i32.load8_u offset=20
    i32.const 1
    i32.ne
    i32.or
    i32.eqz
    if $I25
      get_local $l10
      i32.load offset=13124
      set_local $l11
      get_local $l0
      get_local $l0
      i32.load8_u offset=80
      tee_local $p0
      i32.store8 offset=240
      block $B26
        get_local $l8
        i32.eqz
        if $I27
          get_local $p3
          i32.const 31
          i32.eq
          br_if $B26
          get_local $p0
          i32.const 16843009
          i32.mul
          set_local $l7
          i32.const 0
          set_local $p0
          loop $L28
            get_local $p0
            get_local $l5
            i32.add
            get_local $l7
            i32.store align=1
            get_local $p0
            i32.const 4
            i32.add
            tee_local $p0
            get_local $l1
            i32.lt_s
            br_if $L28
          end
        end
        get_local $l6
        get_local $p3
        i32.const 31
        i32.eq
        i32.or
        br_if $B26
        get_local $l0
        i32.const 240
        i32.add
        get_local $l1
        i32.add
        i32.load8_u
        i32.const 16843009
        i32.mul
        set_local $l7
        get_local $l1
        get_local $l5
        i32.add
        set_local $l14
        i32.const 0
        set_local $p0
        loop $L29
          get_local $p0
          get_local $l14
          i32.add
          get_local $l7
          i32.store align=1
          get_local $p0
          i32.const 4
          i32.add
          tee_local $p0
          get_local $l1
          i32.lt_s
          br_if $L29
        end
      end
      block $B30
        get_local $p1
        i32.const 0
        i32.ne
        get_local $p2
        i32.const 0
        i32.ne
        i32.and
        get_local $p1
        i32.or
        br_if $B30
        get_local $l1
        i32.const 1
        i32.shl
        tee_local $p0
        get_local $l11
        get_local $p2
        i32.sub
        get_local $l18
        i32.shr_s
        tee_local $p1
        get_local $p0
        get_local $l18
        i32.shl
        get_local $p2
        i32.add
        get_local $l11
        i32.lt_s
        select
        get_local $l1
        get_local $p1
        get_local $l11
        get_local $l20
        i32.gt_s
        select
        get_local $l6
        select
        tee_local $p0
        i32.const 1
        i32.lt_s
        br_if $B30
        get_local $l5
        i32.const 0
        get_local $p0
        i32.const 3
        i32.add
        i32.const -4
        i32.and
        call $memset
        drop
      end
      get_local $l0
      get_local $l0
      i32.load8_u offset=240
      i32.store8 offset=80
    end
    block $B31
      block $B32
        block $B33
          block $B34
            block $B35
              block $B36
                block $B37
                  get_local $l6
                  i32.eqz
                  if $I38
                    get_local $l8
                    if $I39
                      get_local $p3
                      i32.const 31
                      i32.eq
                      br_if $B36
                      get_local $l0
                      i32.const 240
                      i32.add
                      get_local $l1
                      i32.add
                      i32.load8_u
                      i32.const 16843009
                      i32.mul
                      set_local $p1
                      get_local $l1
                      get_local $l5
                      i32.add
                      set_local $p2
                      i32.const 0
                      set_local $p0
                      loop $L40
                        get_local $p0
                        get_local $p2
                        i32.add
                        get_local $p1
                        i32.store align=1
                        get_local $p0
                        i32.const 4
                        i32.add
                        tee_local $p0
                        get_local $l1
                        i32.lt_s
                        br_if $L40
                      end
                      br $B36
                    end
                    get_local $l15
                    if $I41
                      get_local $p3
                      i32.const 31
                      i32.eq
                      br_if $B33
                      get_local $l1
                      i32.const 1
                      i32.shl
                      set_local $p1
                      get_local $l0
                      i32.load8_u offset=240
                      i32.const 16843009
                      i32.mul
                      set_local $p2
                      i32.const 0
                      set_local $p0
                      loop $L42
                        get_local $p0
                        get_local $l5
                        i32.add
                        get_local $p2
                        i32.store align=1
                        get_local $p0
                        i32.const 4
                        i32.add
                        tee_local $p0
                        get_local $p1
                        i32.lt_s
                        br_if $L42
                      end
                      br $B34
                    end
                    get_local $l4
                    if $I43
                      get_local $l0
                      get_local $l0
                      i32.load8_u offset=81
                      tee_local $p0
                      i32.store8 offset=240
                      get_local $p3
                      i32.const 31
                      i32.eq
                      br_if $B33
                      get_local $l1
                      i32.const 1
                      i32.shl
                      set_local $p1
                      get_local $p0
                      i32.const 16843009
                      i32.mul
                      set_local $p2
                      i32.const 0
                      set_local $p0
                      loop $L44
                        get_local $p0
                        get_local $l5
                        i32.add
                        get_local $p2
                        i32.store align=1
                        get_local $p0
                        i32.const 4
                        i32.add
                        tee_local $p0
                        get_local $p1
                        i32.lt_s
                        br_if $L44
                      end
                      br $B33
                    end
                    get_local $l9
                    if $I45
                      get_local $l1
                      get_local $l2
                      i32.add
                      tee_local $p1
                      i32.load8_u
                      set_local $p0
                      get_local $p3
                      i32.const 31
                      i32.eq
                      br_if $B32
                      get_local $p0
                      i32.const 16843009
                      i32.mul
                      set_local $p2
                      i32.const 0
                      set_local $p0
                      loop $L46
                        get_local $p0
                        get_local $l2
                        i32.add
                        get_local $p2
                        i32.store align=1
                        get_local $p0
                        i32.const 4
                        i32.add
                        tee_local $p0
                        get_local $l1
                        i32.lt_s
                        br_if $L46
                      end
                      get_local $l0
                      get_local $p1
                      i32.load8_u
                      tee_local $p0
                      i32.store8 offset=240
                      get_local $p3
                      i32.const 31
                      i32.eq
                      br_if $B33
                      get_local $l1
                      i32.const 1
                      i32.shl
                      set_local $p1
                      get_local $p0
                      i32.const 16843009
                      i32.mul
                      set_local $p2
                      i32.const 0
                      set_local $p0
                      loop $L47
                        get_local $p0
                        get_local $l5
                        i32.add
                        get_local $p2
                        i32.store align=1
                        get_local $p0
                        i32.const 4
                        i32.add
                        tee_local $p0
                        get_local $p1
                        i32.lt_s
                        br_if $L47
                      end
                      br $B33
                    end
                    get_local $l0
                    i32.const 128
                    i32.store8 offset=240
                    i32.const 0
                    set_local $l4
                    get_local $p3
                    i32.const 31
                    i32.eq
                    br_if $B35
                    get_local $l2
                    i32.const 128
                    get_local $l1
                    i32.const 1
                    i32.shl
                    tee_local $p0
                    i32.const 4
                    get_local $p0
                    i32.const 4
                    i32.gt_s
                    select
                    i32.const 3
                    i32.add
                    i32.const -4
                    i32.and
                    tee_local $p0
                    call $memset
                    drop
                    get_local $l5
                    i32.const 128
                    get_local $p0
                    call $memset
                    drop
                    i32.const 0
                    set_local $l15
                    br $B37
                  end
                  get_local $p3
                  i32.const 31
                  i32.eq
                  get_local $l8
                  i32.or
                  br_if $B36
                end
                get_local $l1
                get_local $l5
                i32.add
                i32.load8_u
                i32.const 16843009
                i32.mul
                set_local $p1
                i32.const 0
                set_local $p0
                loop $L48
                  get_local $p0
                  get_local $l5
                  i32.add
                  get_local $p1
                  i32.store align=1
                  get_local $p0
                  i32.const 4
                  i32.add
                  tee_local $p0
                  get_local $l1
                  i32.lt_s
                  br_if $L48
                end
              end
              get_local $l15
              br_if $B34
            end
            get_local $l0
            get_local $l0
            i32.load8_u offset=241
            i32.store8 offset=240
          end
          get_local $l4
          get_local $p3
          i32.const 31
          i32.eq
          i32.or
          br_if $B33
          get_local $l0
          i32.load8_u offset=240
          i32.const 16843009
          i32.mul
          set_local $p1
          i32.const 0
          set_local $p0
          loop $L49
            get_local $p0
            get_local $l2
            i32.add
            get_local $p1
            i32.store align=1
            get_local $p0
            i32.const 4
            i32.add
            tee_local $p0
            get_local $l1
            i32.lt_s
            br_if $L49
          end
        end
        get_local $l9
        get_local $p3
        i32.const 31
        i32.eq
        i32.or
        br_if $B31
        get_local $l0
        i32.const 80
        i32.add
        get_local $l1
        i32.add
        i32.load8_u
        i32.const 16843009
        i32.mul
        set_local $p1
        get_local $l1
        get_local $l2
        i32.add
        set_local $p2
        i32.const 0
        set_local $p0
        loop $L50
          get_local $p0
          get_local $p2
          i32.add
          get_local $p1
          i32.store align=1
          get_local $p0
          i32.const 4
          i32.add
          tee_local $p0
          get_local $l1
          i32.lt_s
          br_if $L50
        end
        br $B31
      end
      get_local $l0
      get_local $p0
      i32.store8 offset=240
    end
    get_local $l0
    get_local $l0
    i32.load8_u offset=240
    tee_local $l4
    i32.store8 offset=80
    block $B51
      get_local $l10
      i32.load offset=13112
      br_if $B51
      get_local $p4
      if $I52
        get_local $l10
        i32.load offset=4
        i32.const 3
        i32.ne
        br_if $B51
      end
      get_local $p3
      i32.const 2
      i32.eq
      get_local $l13
      i32.const 1
      i32.eq
      i32.or
      br_if $B51
      get_local $p3
      i32.const 2
      i32.shl
      i32.const 2504
      i32.add
      i32.load
      get_local $l13
      i32.const 10
      i32.sub
      tee_local $p0
      i32.const 10
      get_local $l13
      i32.sub
      get_local $p0
      i32.const -1
      i32.gt_s
      select
      tee_local $p0
      get_local $l13
      i32.const 26
      i32.sub
      tee_local $p1
      i32.const 26
      get_local $l13
      i32.sub
      get_local $p1
      i32.const -1
      i32.gt_s
      select
      tee_local $p1
      get_local $p0
      get_local $p1
      i32.lt_s
      select
      i32.ge_s
      br_if $B51
      get_local $l0
      i32.const 1
      i32.or
      set_local $p1
      block $B53
        get_local $p3
        i32.const 5
        i32.ne
        get_local $p4
        i32.or
        br_if $B53
        get_local $l10
        i32.load8_u offset=13061
        i32.eqz
        br_if $B53
        get_local $l0
        i32.load8_u offset=144
        tee_local $l6
        get_local $l4
        i32.add
        get_local $l0
        i32.load8_u offset=112
        i32.const 1
        i32.shl
        i32.sub
        tee_local $p0
        get_local $p0
        i32.const 31
        i32.shr_s
        tee_local $p0
        i32.add
        get_local $p0
        i32.xor
        i32.const 7
        i32.gt_u
        br_if $B53
        get_local $l0
        i32.load8_u offset=304
        tee_local $l9
        get_local $l4
        i32.add
        get_local $l0
        i32.load8_u offset=272
        i32.const 1
        i32.shl
        i32.sub
        tee_local $p0
        get_local $p0
        i32.const 31
        i32.shr_s
        tee_local $p0
        i32.add
        get_local $p0
        i32.xor
        i32.const 7
        i32.gt_u
        br_if $B53
        get_local $l0
        get_local $l6
        i32.store8 offset=64
        get_local $l0
        get_local $l4
        i32.store8
        i32.const 0
        set_local $p0
        loop $L54
          get_local $p0
          get_local $p1
          i32.add
          get_local $p0
          i32.const 1
          i32.add
          tee_local $p2
          get_local $l6
          i32.mul
          i32.const 63
          get_local $p0
          i32.sub
          get_local $l4
          i32.mul
          i32.add
          i32.const 32
          i32.add
          i32.const 6
          i32.shr_u
          i32.store8
          get_local $p2
          tee_local $p0
          i32.const 63
          i32.ne
          br_if $L54
        end
        get_local $l0
        get_local $l4
        i32.const 63
        i32.mul
        get_local $l9
        i32.add
        i32.const 32
        i32.add
        i32.const 6
        i32.shr_u
        i32.store8 offset=241
        i32.const 1
        set_local $p0
        loop $L55
          get_local $p0
          get_local $l5
          i32.add
          i32.const 63
          get_local $p0
          i32.sub
          get_local $l4
          i32.mul
          get_local $p0
          i32.const 1
          i32.add
          tee_local $p0
          get_local $l0
          i32.load8_u offset=304
          i32.mul
          i32.add
          i32.const 32
          i32.add
          i32.const 6
          i32.shr_u
          i32.store8
          get_local $p0
          i32.const 63
          i32.ne
          br_if $L55
        end
        get_local $p1
        set_local $l2
        br $B51
      end
      get_local $l0
      i32.const 160
      i32.add
      i32.const 1
      i32.or
      set_local $l6
      get_local $l1
      i32.const 1
      i32.shl
      tee_local $p0
      get_local $l0
      i32.const 160
      i32.add
      i32.add
      get_local $l0
      i32.const 240
      i32.add
      get_local $p0
      i32.add
      i32.load8_u
      i32.store8
      get_local $p0
      get_local $l0
      i32.add
      get_local $l0
      i32.const 80
      i32.add
      get_local $p0
      i32.add
      i32.load8_u
      i32.store8
      get_local $p0
      i32.const 2
      i32.sub
      set_local $p0
      get_local $p3
      i32.const 31
      i32.eq
      tee_local $l9
      i32.eqz
      if $I56
        get_local $p0
        set_local $p2
        loop $L57
          get_local $p2
          get_local $l6
          i32.add
          get_local $l0
          i32.const 240
          i32.add
          get_local $p2
          i32.add
          i32.load8_u
          get_local $p2
          get_local $l5
          i32.add
          tee_local $l8
          i32.load8_u offset=1
          get_local $l8
          i32.load8_u
          i32.const 1
          i32.shl
          i32.add
          i32.add
          i32.const 2
          i32.add
          i32.const 2
          i32.shr_u
          i32.store8
          get_local $p2
          i32.const 0
          i32.gt_s
          set_local $l8
          get_local $p2
          i32.const 1
          i32.sub
          set_local $p2
          get_local $l8
          br_if $L57
        end
      end
      get_local $l0
      get_local $l0
      i32.load8_u offset=81
      get_local $l0
      i32.load8_u offset=241
      get_local $l4
      i32.const 1
      i32.shl
      i32.add
      i32.add
      i32.const 2
      i32.add
      i32.const 2
      i32.shr_u
      tee_local $p2
      i32.store8 offset=160
      get_local $l0
      get_local $p2
      i32.store8
      get_local $l9
      i32.eqz
      if $I58
        loop $L59
          get_local $p0
          get_local $p1
          i32.add
          get_local $l0
          i32.const 80
          i32.add
          get_local $p0
          i32.add
          i32.load8_u
          get_local $p0
          get_local $l2
          i32.add
          tee_local $p2
          i32.load8_u offset=1
          get_local $p2
          i32.load8_u
          i32.const 1
          i32.shl
          i32.add
          i32.add
          i32.const 2
          i32.add
          i32.const 2
          i32.shr_u
          i32.store8
          get_local $p0
          i32.const 0
          i32.gt_s
          set_local $p2
          get_local $p0
          i32.const 1
          i32.sub
          set_local $p0
          get_local $p2
          br_if $L59
        end
      end
      get_local $p1
      set_local $l2
      get_local $l6
      set_local $l5
    end
    block $B60
      block $B61
        block $B62
          block $B63
            get_local $l13
            br_table $B63 $B62 $B61
          end
          get_local $l3
          get_local $l2
          get_local $l5
          get_local $l12
          get_local $p3
          call $pred_planar_8
          br $B60
        end
        get_local $l3
        get_local $l2
        get_local $l5
        get_local $l12
        get_local $p3
        get_local $p4
        call $pred_dc_8
        br $B60
      end
      get_local $l3
      get_local $l2
      get_local $l5
      get_local $l12
      get_local $p4
      get_local $l13
      get_local $l1
      block $B64 (result i32)
        i32.const 0
        get_local $l10
        i32.load offset=13104
        i32.eqz
        br_if $B64
        drop
        get_local $l17
        i32.const 31256
        i32.add
        i32.load8_u
        i32.const 0
        i32.ne
      end
      call $pred_angular_8
    end
    get_local $l0
    i32.const 320
    i32.add
    set_global $g0)
  (func $pred_planar_8 (type $t8) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32)
    get_local $p4
    i32.const 31
    i32.ne
    if $I0
      i32.const 1
      get_local $p4
      i32.shl
      tee_local $l0
      i32.const 1
      get_local $l0
      i32.const 1
      i32.gt_s
      select
      set_local $l1
      get_local $p4
      i32.const 1
      i32.add
      set_local $l4
      get_local $p2
      get_local $l0
      i32.add
      set_local $l5
      get_local $p1
      get_local $l0
      i32.add
      set_local $l6
      get_local $l0
      i32.const 1
      i32.sub
      set_local $l2
      i32.const 0
      set_local $p4
      loop $L1
        get_local $p3
        get_local $p4
        i32.mul
        set_local $l7
        get_local $p4
        i32.const 1
        i32.add
        set_local $l3
        get_local $l2
        get_local $p4
        i32.sub
        set_local $l8
        get_local $p2
        get_local $p4
        i32.add
        set_local $l9
        i32.const 0
        set_local $p4
        loop $L2
          get_local $p0
          get_local $p4
          get_local $l7
          i32.add
          i32.add
          get_local $l9
          i32.load8_u
          get_local $l2
          get_local $p4
          i32.sub
          i32.mul
          get_local $l0
          i32.add
          get_local $p4
          i32.const 1
          i32.add
          tee_local $l10
          get_local $l6
          i32.load8_u
          i32.mul
          i32.add
          get_local $l8
          get_local $p1
          get_local $p4
          i32.add
          i32.load8_u
          i32.mul
          i32.add
          get_local $l3
          get_local $l5
          i32.load8_u
          i32.mul
          i32.add
          get_local $l4
          i32.shr_s
          i32.store8
          get_local $l10
          tee_local $p4
          get_local $l1
          i32.ne
          br_if $L2
        end
        get_local $l3
        tee_local $p4
        get_local $l1
        i32.ne
        br_if $L1
      end
    end)
  (func $pred_dc_8 (type $t6) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32)
    i32.const 1
    get_local $p4
    i32.shl
    set_local $l1
    i32.const -1
    set_local $l2
    block $B0
      get_local $p4
      i32.const 31
      i32.eq
      br_if $B0
      get_local $l1
      i32.const 1
      get_local $l1
      i32.const 1
      i32.gt_s
      select
      set_local $l3
      get_local $l1
      set_local $l2
      loop $L1
        get_local $p1
        get_local $l0
        i32.add
        i32.load8_u
        get_local $l2
        get_local $p2
        get_local $l0
        i32.add
        i32.load8_u
        i32.add
        i32.add
        set_local $l2
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $l3
        i32.ne
        br_if $L1
      end
      get_local $l2
      get_local $p4
      i32.const 1
      i32.add
      i32.shr_s
      set_local $l2
      get_local $p4
      i32.const 31
      i32.eq
      br_if $B0
      get_local $l2
      i32.const 16843009
      i32.mul
      set_local $l3
      get_local $l1
      i32.const 1
      get_local $l1
      i32.const 1
      i32.gt_s
      select
      set_local $l4
      i32.const 0
      set_local $p4
      loop $L2
        get_local $p3
        get_local $p4
        i32.mul
        set_local $l5
        i32.const 0
        set_local $l0
        loop $L3
          get_local $p0
          get_local $l0
          get_local $l5
          i32.add
          i32.add
          get_local $l3
          i32.store align=1
          get_local $l0
          i32.const 4
          i32.add
          tee_local $l0
          get_local $l1
          i32.lt_s
          br_if $L3
        end
        get_local $p4
        i32.const 1
        i32.add
        tee_local $p4
        get_local $l4
        i32.ne
        br_if $L2
      end
    end
    block $B4
      get_local $p5
      get_local $l1
      i32.const 31
      i32.gt_s
      i32.or
      br_if $B4
      get_local $p0
      get_local $p1
      i32.load8_u
      get_local $p2
      i32.load8_u
      get_local $l2
      i32.const 1
      i32.shl
      i32.add
      i32.add
      i32.const 2
      i32.add
      i32.const 2
      i32.shr_u
      i32.store8
      get_local $l1
      i32.const 2
      i32.lt_s
      br_if $B4
      get_local $l2
      i32.const 3
      i32.mul
      i32.const 2
      i32.add
      set_local $p4
      i32.const 1
      set_local $l0
      loop $L5
        get_local $p0
        get_local $l0
        i32.add
        get_local $p4
        get_local $p1
        get_local $l0
        i32.add
        i32.load8_u
        i32.add
        i32.const 2
        i32.shr_u
        i32.store8
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $l1
        i32.ne
        br_if $L5
      end
      get_local $l1
      i32.const 2
      i32.lt_s
      br_if $B4
      get_local $l2
      i32.const 3
      i32.mul
      i32.const 2
      i32.add
      set_local $p1
      i32.const 1
      set_local $l0
      loop $L6
        get_local $p0
        get_local $p3
        get_local $l0
        i32.mul
        i32.add
        get_local $p1
        get_local $p2
        get_local $l0
        i32.add
        i32.load8_u
        i32.add
        i32.const 2
        i32.shr_u
        i32.store8
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $l1
        i32.ne
        br_if $L6
      end
    end)
  (func $pred_angular_8 (type $t16) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32)
    get_global $g0
    i32.const 112
    i32.sub
    tee_local $l8
    set_global $g0
    get_local $p5
    i32.const 2526
    i32.add
    i32.load8_s
    tee_local $l9
    get_local $p6
    i32.mul
    tee_local $l1
    i32.const 5
    i32.shr_s
    set_local $l0
    get_local $p6
    get_local $l8
    i32.add
    set_local $l5
    block $B0
      get_local $p5
      i32.const 18
      i32.ge_s
      if $I1
        get_local $p1
        i32.const 1
        i32.sub
        set_local $l2
        get_local $l1
        i32.const -33
        i32.gt_s
        get_local $p5
        i32.const 11
        i32.sub
        tee_local $l1
        i32.const 14
        i32.gt_u
        i32.or
        i32.eqz
        if $I2
          get_local $p6
          i32.const 0
          i32.ge_s
          if $I3
            get_local $l5
            get_local $l2
            get_local $p6
            i32.const 4
            i32.add
            i32.const -4
            i32.and
            call $memcpy
            drop
          end
          get_local $l0
          i32.const -1
          get_local $l0
          i32.const -1
          i32.gt_s
          select
          set_local $l2
          get_local $l1
          i32.const 1
          i32.shl
          i32.const 2576
          i32.add
          i32.load16_s
          set_local $l1
          loop $L4
            get_local $l0
            get_local $l5
            i32.add
            get_local $l0
            get_local $l1
            i32.mul
            i32.const 128
            i32.add
            i32.const 8
            i32.shr_s
            get_local $p2
            i32.add
            i32.const 1
            i32.sub
            i32.load8_u
            i32.store8
            get_local $l0
            get_local $l2
            i32.ne
            set_local $l3
            get_local $l0
            i32.const 1
            i32.add
            set_local $l0
            get_local $l3
            br_if $L4
          end
          get_local $l5
          set_local $l2
        end
        get_local $p6
        i32.const 1
        i32.ge_s
        if $I5
          i32.const 0
          set_local $l0
          loop $L6
            get_local $l0
            i32.const 1
            i32.add
            tee_local $l5
            get_local $l9
            i32.mul
            tee_local $l3
            i32.const 5
            i32.shr_s
            set_local $l1
            block $B7
              get_local $l3
              i32.const 31
              i32.and
              tee_local $l3
              if $I8
                get_local $p3
                get_local $l0
                i32.mul
                set_local $l6
                i32.const 32
                get_local $l3
                i32.sub
                set_local $l7
                i32.const 0
                set_local $l0
                loop $L9
                  get_local $p0
                  get_local $l0
                  get_local $l6
                  i32.add
                  i32.add
                  get_local $l7
                  get_local $l0
                  get_local $l1
                  i32.add
                  get_local $l2
                  i32.add
                  tee_local $l4
                  i32.load8_u offset=1
                  i32.mul
                  get_local $l3
                  get_local $l4
                  i32.load8_u offset=2
                  i32.mul
                  i32.add
                  i32.const 16
                  i32.add
                  i32.const 5
                  i32.shr_u
                  i32.store8
                  get_local $p0
                  get_local $l0
                  i32.const 1
                  i32.or
                  tee_local $l4
                  get_local $l6
                  i32.add
                  i32.add
                  get_local $l7
                  get_local $l1
                  get_local $l4
                  i32.add
                  get_local $l2
                  i32.add
                  tee_local $l4
                  i32.load8_u offset=1
                  i32.mul
                  get_local $l3
                  get_local $l4
                  i32.load8_u offset=2
                  i32.mul
                  i32.add
                  i32.const 16
                  i32.add
                  i32.const 5
                  i32.shr_u
                  i32.store8
                  get_local $p0
                  get_local $l0
                  i32.const 2
                  i32.or
                  tee_local $l4
                  get_local $l6
                  i32.add
                  i32.add
                  get_local $l7
                  get_local $l1
                  get_local $l4
                  i32.add
                  get_local $l2
                  i32.add
                  tee_local $l4
                  i32.load8_u offset=1
                  i32.mul
                  get_local $l3
                  get_local $l4
                  i32.load8_u offset=2
                  i32.mul
                  i32.add
                  i32.const 16
                  i32.add
                  i32.const 5
                  i32.shr_u
                  i32.store8
                  get_local $p0
                  get_local $l0
                  i32.const 3
                  i32.or
                  tee_local $l4
                  get_local $l6
                  i32.add
                  i32.add
                  get_local $l7
                  get_local $l1
                  get_local $l4
                  i32.add
                  get_local $l2
                  i32.add
                  tee_local $l4
                  i32.load8_u offset=1
                  i32.mul
                  get_local $l3
                  get_local $l4
                  i32.load8_u offset=2
                  i32.mul
                  i32.add
                  i32.const 16
                  i32.add
                  i32.const 5
                  i32.shr_u
                  i32.store8
                  get_local $l0
                  i32.const 4
                  i32.add
                  tee_local $l0
                  get_local $p6
                  i32.lt_s
                  br_if $L9
                end
                br $B7
              end
              get_local $p3
              get_local $l0
              i32.mul
              set_local $l3
              get_local $l1
              i32.const 1
              i32.add
              set_local $l1
              i32.const 0
              set_local $l0
              loop $L10
                get_local $p0
                get_local $l0
                get_local $l3
                i32.add
                i32.add
                get_local $l2
                get_local $l0
                get_local $l1
                i32.add
                i32.add
                i32.load align=1
                i32.store align=1
                get_local $l0
                i32.const 4
                i32.add
                tee_local $l0
                get_local $p6
                i32.lt_s
                br_if $L10
              end
            end
            get_local $l5
            tee_local $l0
            get_local $p6
            i32.ne
            br_if $L6
          end
        end
        get_local $p4
        get_local $p5
        i32.const 26
        i32.ne
        i32.or
        get_local $p6
        i32.const 31
        i32.gt_s
        get_local $p7
        i32.or
        i32.or
        get_local $p6
        i32.const 1
        i32.lt_s
        i32.or
        br_if $B0
        get_local $p2
        i32.const 1
        i32.sub
        set_local $p5
        i32.const 0
        set_local $l0
        loop $L11
          get_local $p0
          get_local $p3
          get_local $l0
          i32.mul
          i32.add
          get_local $p1
          i32.load8_u
          get_local $p2
          get_local $l0
          i32.add
          i32.load8_u
          get_local $p5
          i32.load8_u
          i32.sub
          i32.const 1
          i32.shr_s
          i32.add
          tee_local $p4
          i32.const -1
          i32.const 0
          get_local $p4
          i32.const 0
          i32.gt_s
          select
          get_local $p4
          i32.const 256
          i32.lt_u
          select
          i32.store8
          get_local $l0
          i32.const 1
          i32.add
          tee_local $l0
          get_local $p6
          i32.ne
          br_if $L11
        end
        br $B0
      end
      get_local $p2
      i32.const 1
      i32.sub
      set_local $l2
      get_local $l1
      i32.const -33
      i32.gt_s
      get_local $p5
      i32.const 11
      i32.sub
      tee_local $l1
      i32.const 14
      i32.gt_u
      i32.or
      i32.eqz
      if $I12
        get_local $p6
        i32.const 0
        i32.ge_s
        if $I13
          get_local $l5
          get_local $l2
          get_local $p6
          i32.const 4
          i32.add
          i32.const -4
          i32.and
          call $memcpy
          drop
        end
        get_local $l0
        i32.const -1
        get_local $l0
        i32.const -1
        i32.gt_s
        select
        set_local $l2
        get_local $l1
        i32.const 1
        i32.shl
        i32.const 2576
        i32.add
        i32.load16_s
        set_local $l1
        loop $L14
          get_local $l0
          get_local $l5
          i32.add
          get_local $l0
          get_local $l1
          i32.mul
          i32.const 128
          i32.add
          i32.const 8
          i32.shr_s
          get_local $p1
          i32.add
          i32.const 1
          i32.sub
          i32.load8_u
          i32.store8
          get_local $l0
          get_local $l2
          i32.ne
          set_local $l3
          get_local $l0
          i32.const 1
          i32.add
          set_local $l0
          get_local $l3
          br_if $L14
        end
        get_local $l5
        set_local $l2
      end
      get_local $p6
      i32.const 1
      i32.ge_s
      if $I15
        i32.const 0
        set_local $l5
        loop $L16
          get_local $l5
          i32.const 1
          i32.add
          tee_local $l1
          get_local $l9
          i32.mul
          tee_local $l0
          i32.const 5
          i32.shr_s
          set_local $l3
          block $B17
            get_local $l0
            i32.const 31
            i32.and
            tee_local $l6
            i32.eqz
            if $I18
              i32.const 0
              set_local $l0
              loop $L19
                get_local $p0
                get_local $p3
                get_local $l0
                i32.mul
                get_local $l5
                i32.add
                i32.add
                get_local $l2
                get_local $l0
                i32.const 1
                i32.add
                tee_local $l0
                get_local $l3
                i32.add
                i32.add
                i32.load8_u
                i32.store8
                get_local $p6
                get_local $l0
                i32.ne
                br_if $L19
              end
              br $B17
            end
            i32.const 32
            get_local $l6
            i32.sub
            set_local $l7
            i32.const 0
            set_local $l0
            loop $L20
              get_local $p0
              get_local $p3
              get_local $l0
              i32.mul
              get_local $l5
              i32.add
              i32.add
              get_local $l7
              get_local $l0
              get_local $l3
              i32.add
              get_local $l2
              i32.add
              tee_local $l4
              i32.load8_u offset=1
              i32.mul
              get_local $l6
              get_local $l4
              i32.load8_u offset=2
              i32.mul
              i32.add
              i32.const 16
              i32.add
              i32.const 5
              i32.shr_u
              i32.store8
              get_local $l0
              i32.const 1
              i32.add
              tee_local $l0
              get_local $p6
              i32.ne
              br_if $L20
            end
          end
          get_local $l1
          tee_local $l5
          get_local $p6
          i32.ne
          br_if $L16
        end
      end
      get_local $p4
      get_local $p5
      i32.const 10
      i32.ne
      i32.or
      get_local $p6
      i32.const 31
      i32.gt_s
      get_local $p7
      i32.or
      i32.or
      get_local $p6
      i32.const 1
      i32.lt_s
      i32.or
      br_if $B0
      get_local $p1
      i32.const 1
      i32.sub
      set_local $p3
      i32.const 0
      set_local $l0
      loop $L21
        get_local $p0
        get_local $l0
        i32.add
        get_local $p2
        i32.load8_u
        get_local $p1
        get_local $l0
        i32.add
        i32.load8_u
        get_local $p3
        i32.load8_u
        i32.sub
        i32.const 1
        i32.shr_s
        i32.add
        tee_local $p4
        i32.const -1
        i32.const 0
        get_local $p4
        i32.const 0
        i32.gt_s
        select
        get_local $p4
        i32.const 256
        i32.lt_u
        select
        i32.store8
        get_local $p0
        get_local $l0
        i32.const 1
        i32.or
        tee_local $p4
        i32.add
        get_local $p2
        i32.load8_u
        get_local $p1
        get_local $p4
        i32.add
        i32.load8_u
        get_local $p3
        i32.load8_u
        i32.sub
        i32.const 1
        i32.shr_s
        i32.add
        tee_local $p4
        i32.const -1
        i32.const 0
        get_local $p4
        i32.const 0
        i32.gt_s
        select
        get_local $p4
        i32.const 256
        i32.lt_u
        select
        i32.store8
        get_local $p0
        get_local $l0
        i32.const 2
        i32.or
        tee_local $p4
        i32.add
        get_local $p2
        i32.load8_u
        get_local $p1
        get_local $p4
        i32.add
        i32.load8_u
        get_local $p3
        i32.load8_u
        i32.sub
        i32.const 1
        i32.shr_s
        i32.add
        tee_local $p4
        i32.const -1
        i32.const 0
        get_local $p4
        i32.const 0
        i32.gt_s
        select
        get_local $p4
        i32.const 256
        i32.lt_u
        select
        i32.store8
        get_local $p0
        get_local $l0
        i32.const 3
        i32.or
        tee_local $p4
        i32.add
        get_local $p2
        i32.load8_u
        get_local $p1
        get_local $p4
        i32.add
        i32.load8_u
        get_local $p3
        i32.load8_u
        i32.sub
        i32.const 1
        i32.shr_s
        i32.add
        tee_local $p4
        i32.const -1
        i32.const 0
        get_local $p4
        i32.const 0
        i32.gt_s
        select
        get_local $p4
        i32.const 256
        i32.lt_u
        select
        i32.store8
        get_local $l0
        i32.const 4
        i32.add
        tee_local $l0
        get_local $p6
        i32.lt_s
        br_if $L21
      end
    end
    get_local $l8
    i32.const 112
    i32.add
    set_global $g0)
  (func $ff_hevc_unref_frame (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32)
    block $B0
      get_local $p1
      i32.load
      tee_local $l0
      i32.eqz
      br_if $B0
      get_local $l0
      i32.load offset=304
      i32.eqz
      br_if $B0
      get_local $p1
      get_local $p1
      i32.load8_u offset=46
      get_local $p2
      i32.const -1
      i32.xor
      i32.and
      tee_local $p2
      i32.store8 offset=46
      get_local $p2
      br_if $B0
      get_local $p0
      i32.load offset=4
      drop
      get_local $p1
      i32.load offset=4
      tee_local $p0
      if $I1
        get_local $p0
        call $av_frame_unref
      end
      get_local $p1
      i32.const 0
      i32.store offset=24
    end)
  (func $ff_hevc_clear_refs (type $t1) (param $p0 i32)
    get_local $p0
    get_local $p0
    i32.const 2524
    i32.add
    i32.const 6
    call $ff_hevc_unref_frame)
  (func $ff_hevc_set_new_ref (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32)
    block $B0 (result i32)
      block $B1
        get_local $p0
        i32.load offset=2524
        i32.load offset=304
        i32.eqz
        br_if $B1
        get_local $p0
        i32.const 2568
        i32.add
        i32.load16_u
        get_local $p0
        i32.load16_u offset=4364
        i32.ne
        br_if $B1
        i32.const -1094995529
        get_local $p0
        i32.const 2544
        i32.add
        i32.load
        get_local $p2
        i32.eq
        br_if $B0
        drop
      end
      get_local $p0
      call $alloc_frame
      tee_local $l0
      i32.eqz
      if $I2
        i32.const -48
        return
      end
      get_local $p1
      get_local $l0
      i32.load
      i32.store
      get_local $p0
      get_local $l0
      i32.store offset=2520
      get_local $p0
      i32.const 1450
      i32.add
      i32.load8_u
      set_local $p1
      get_local $l0
      get_local $p2
      i32.store offset=20
      get_local $l0
      i32.const 3
      i32.const 2
      get_local $p1
      select
      i32.store8 offset=46
      get_local $l0
      get_local $p0
      i32.load16_u offset=4364
      i32.store16 offset=44
      get_local $l0
      get_local $p0
      i32.load offset=200
      tee_local $p0
      i64.load offset=20 align=4
      i64.store offset=28 align=4
      get_local $l0
      get_local $p0
      i64.load offset=28 align=4
      i64.store offset=36 align=4
      i32.const 0
    end)
  (func $alloc_frame (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    block $B0
      get_local $p0
      i32.load offset=2524
      i32.load offset=304
      br_if $B0
      get_local $p0
      i32.const 2528
      i32.add
      tee_local $l0
      get_local $p0
      i32.load offset=4
      tee_local $l1
      i32.store offset=4
      get_local $l1
      get_local $l0
      i32.load
      call $get_buffer_internal
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $p0
      i32.const 2524
      i32.add
      set_local $l2
      get_local $p0
      i32.const 2540
      i32.add
      get_local $p0
      i32.load offset=200
      tee_local $l0
      i32.load offset=13132
      get_local $l0
      i32.load offset=13128
      i32.mul
      i32.store
      get_local $p0
      i32.load offset=2524
      tee_local $l0
      get_local $p0
      i32.load offset=4520
      tee_local $p0
      i32.const 1
      i32.sub
      i32.const 2
      i32.lt_u
      i32.store offset=240
      get_local $l0
      get_local $p0
      i32.const 1
      i32.eq
      i32.store offset=244
    end
    get_local $l2)
  (func $ff_hevc_output_frame (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p0
    i32.const 2524
    i32.add
    set_local $l2
    loop $L0
      block $B1
        get_local $p0
        i32.load8_u offset=2046
        i32.const 1
        i32.ne
        br_if $B1
        get_local $p0
        i32.load8_u offset=2570
        i32.const 8
        i32.and
        br_if $B1
        get_local $p0
        i32.load offset=2544
        get_local $p0
        i32.load offset=2572
        i32.eq
        br_if $B1
        get_local $p0
        i32.load16_u offset=2568
        get_local $p0
        i32.load16_u offset=4366
        i32.ne
        br_if $B1
        get_local $p0
        get_local $l2
        i32.const 1
        call $ff_hevc_unref_frame
      end
      i32.const 1
      set_local $l0
      i32.const 0
      set_local $l3
      get_local $p0
      i32.load8_u offset=2570
      i32.const 1
      i32.and
      if $I2
        get_local $p0
        i32.load16_u offset=2568
        tee_local $l0
        get_local $p0
        i32.load16_u offset=4366
        tee_local $l1
        i32.eq
        set_local $l3
        get_local $l0
        get_local $l1
        i32.ne
        set_local $l0
      end
      block $B3
        block $B4
          get_local $p2
          br_if $B4
          get_local $p0
          i32.load16_u offset=4366
          get_local $p0
          i32.load16_u offset=4364
          i32.ne
          br_if $B4
          get_local $p0
          i32.load offset=200
          tee_local $l1
          i32.eqz
          br_if $B4
          get_local $l3
          get_local $l1
          i32.load offset=72
          i32.const 12
          i32.mul
          get_local $l1
          i32.add
          i32.load offset=68
          i32.le_s
          br_if $B3
        end
        get_local $l0
        i32.eqz
        if $I5
          get_local $p1
          get_local $p0
          i32.load offset=2524
          call $av_frame_ref
          set_local $p1
          get_local $p0
          get_local $l2
          i32.const 9
          i32.const 1
          get_local $p0
          i32.load8_u offset=2570
          i32.const 8
          i32.and
          select
          call $ff_hevc_unref_frame
          get_local $p1
          i32.const 1
          get_local $p1
          i32.const 0
          i32.lt_s
          select
          return
        end
        get_local $p0
        i32.load16_u offset=4366
        tee_local $l0
        get_local $p0
        i32.load16_u offset=4364
        i32.eq
        br_if $B3
        get_local $p0
        get_local $l0
        i32.const 1
        i32.add
        i32.const 255
        i32.and
        i32.store16 offset=4366
        br $L0
      end
    end
    i32.const 0)
  (func $hevc_transform_init (type $t11)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    i32.const 3744
    i32.load8_u
    i32.eqz
    if $I0
      loop $L1
        i32.const 0
        set_local $l2
        loop $L2
          get_local $l1
          i32.const 5
          i32.shl
          get_local $l2
          i32.add
          i32.const 3744
          i32.add
          i32.const 64
          get_local $l2
          i32.const 1
          i32.shl
          i32.const 1
          i32.or
          get_local $l1
          i32.mul
          i32.const 127
          i32.and
          tee_local $l0
          i32.const -64
          i32.add
          get_local $l0
          get_local $l0
          i32.const 63
          i32.gt_u
          tee_local $l3
          select
          tee_local $l0
          i32.sub
          get_local $l0
          get_local $l0
          i32.const 31
          i32.gt_s
          tee_local $l0
          select
          i32.const 2608
          i32.add
          i32.load8_u
          i32.const 0
          i32.const -1
          i32.const 1
          get_local $l3
          select
          tee_local $l3
          i32.sub
          get_local $l3
          get_local $l0
          select
          i32.mul
          i32.store8
          get_local $l2
          i32.const 1
          i32.add
          tee_local $l2
          i32.const 32
          i32.ne
          br_if $L2
        end
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 32
        i32.ne
        br_if $L1
      end
    end)
  (func $ff_hevc_dsp_init (type $t4) (param $p0 i32) (param $p1 i32)
    get_local $p0
    i32.const 6
    i32.store offset=1704
    get_local $p0
    i32.const 7
    i32.store offset=1700
    get_local $p0
    i32.const 8
    i32.store offset=1696
    get_local $p0
    i32.const 9
    i32.store offset=1692
    get_local $p0
    i32.const 6
    i32.store offset=1688
    get_local $p0
    i32.const 7
    i32.store offset=1684
    get_local $p0
    i32.const 8
    i32.store offset=1680
    get_local $p0
    i32.const 9
    i32.store offset=1676
    get_local $p0
    i32.const 10
    i32.store offset=68
    get_local $p0
    i32.const 11
    i32.store offset=64
    get_local $p0
    i32.const 12
    i32.store offset=48
    get_local $p0
    i32.const 13
    i32.store offset=32
    get_local $p0
    i32.const 14
    i32.store offset=28
    get_local $p0
    i32.const 15
    i32.store offset=24
    get_local $p0
    i32.const 16
    i32.store offset=20
    get_local $p0
    i32.const 17
    i32.store offset=4
    get_local $p0
    i32.const 18
    i32.store
    get_local $p0
    i32.const 19
    i32.store offset=72
    get_local $p0
    i32.const 20
    i32.store offset=60
    get_local $p0
    i32.const 21
    i32.store offset=56
    get_local $p0
    i32.const 22
    i32.store offset=52
    get_local $p0
    i32.const 23
    i32.store offset=44
    get_local $p0
    i32.const 24
    i32.store offset=40
    get_local $p0
    i32.const 25
    i32.store offset=36
    get_local $p0
    i32.const 26
    i32.store offset=16
    get_local $p0
    i32.const 27
    i32.store offset=12
    get_local $p0
    i32.const 28
    i32.store offset=8)
  (func $hevc_v_loop_filter_chroma_8 (type $t8) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32)
    get_local $p0
    i32.const 1
    get_local $p1
    get_local $p2
    get_local $p3
    get_local $p4
    call $hevc_loop_filter_chroma_8)
  (func $hevc_h_loop_filter_chroma_8 (type $t8) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32)
    get_local $p0
    get_local $p1
    i32.const 1
    get_local $p2
    get_local $p3
    get_local $p4
    call $hevc_loop_filter_chroma_8)
  (func $hevc_v_loop_filter_luma_8 (type $t6) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32)
    get_local $p0
    i32.const 1
    get_local $p1
    get_local $p2
    get_local $p3
    get_local $p4
    get_local $p5
    call $hevc_loop_filter_luma_8)
  (func $hevc_h_loop_filter_luma_8 (type $t6) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32)
    get_local $p0
    get_local $p1
    i32.const 1
    get_local $p2
    get_local $p3
    get_local $p4
    get_local $p5
    call $hevc_loop_filter_luma_8)
  (func $sao_edge_filter_0_8 (type $t15) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (param $p8 i32) (param $p9 i32) (param $p10 i32) (param $p11 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p4
    get_local $p8
    i32.const 10
    i32.mul
    i32.add
    set_local $p11
    block $B0
      block $B1
        get_local $p4
        get_local $p8
        i32.const 2
        i32.shl
        i32.add
        i32.load offset=100
        tee_local $l2
        i32.const 1
        i32.eq
        if $I2
          i32.const 0
          set_local $p10
          br $B1
        end
        block $B3
          get_local $p5
          i32.load
          i32.eqz
          if $I4
            i32.const 0
            set_local $p10
            br $B3
          end
          i32.const 1
          set_local $p10
          get_local $p7
          i32.const 1
          i32.lt_s
          br_if $B3
          get_local $p11
          i32.load16_s offset=112
          set_local $l0
          i32.const 0
          set_local $p9
          loop $L5
            get_local $p0
            get_local $p2
            get_local $p9
            i32.mul
            i32.add
            get_local $p1
            get_local $p3
            get_local $p9
            i32.mul
            i32.add
            i32.load8_u
            get_local $l0
            i32.add
            tee_local $p10
            i32.const -1
            i32.const 0
            get_local $p10
            i32.const 0
            i32.gt_s
            select
            get_local $p10
            i32.const 256
            i32.lt_u
            select
            i32.store8
            i32.const 1
            set_local $p10
            get_local $p9
            i32.const 1
            i32.add
            tee_local $p9
            get_local $p7
            i32.ne
            br_if $L5
          end
        end
        block $B6
          get_local $p5
          i32.load offset=8
          i32.eqz
          br_if $B6
          get_local $p6
          i32.const 1
          i32.sub
          set_local $p6
          get_local $p7
          i32.const 1
          i32.lt_s
          br_if $B6
          get_local $p11
          i32.load16_s offset=112
          set_local $l3
          i32.const 0
          set_local $p9
          loop $L7
            get_local $p0
            get_local $p2
            get_local $p9
            i32.mul
            get_local $p6
            i32.add
            i32.add
            get_local $p1
            get_local $p3
            get_local $p9
            i32.mul
            get_local $p6
            i32.add
            i32.add
            i32.load8_u
            get_local $l3
            i32.add
            tee_local $l0
            i32.const -1
            i32.const 0
            get_local $l0
            i32.const 0
            i32.gt_s
            select
            get_local $l0
            i32.const 256
            i32.lt_u
            select
            i32.store8
            get_local $p9
            i32.const 1
            i32.add
            tee_local $p9
            get_local $p7
            i32.ne
            br_if $L7
          end
        end
        get_local $l2
        br_if $B1
        br $B0
      end
      block $B8
        get_local $p5
        i32.load offset=4
        i32.eqz
        br_if $B8
        i32.const 1
        set_local $l1
        get_local $p6
        get_local $p10
        i32.le_s
        br_if $B8
        get_local $p11
        i32.load16_s offset=112
        set_local $l0
        get_local $p10
        set_local $p9
        loop $L9
          get_local $p0
          get_local $p9
          i32.add
          get_local $p1
          get_local $p9
          i32.add
          i32.load8_u
          get_local $l0
          i32.add
          tee_local $l1
          i32.const -1
          i32.const 0
          get_local $l1
          i32.const 0
          i32.gt_s
          select
          get_local $l1
          i32.const 256
          i32.lt_u
          select
          i32.store8
          i32.const 1
          set_local $l1
          get_local $p9
          i32.const 1
          i32.add
          tee_local $p9
          get_local $p6
          i32.ne
          br_if $L9
        end
      end
      get_local $p5
      i32.load offset=12
      i32.eqz
      br_if $B0
      get_local $p7
      i32.const 1
      i32.sub
      set_local $p7
      get_local $p6
      get_local $p10
      i32.le_s
      br_if $B0
      get_local $p3
      get_local $p7
      i32.mul
      set_local $l0
      get_local $p2
      get_local $p7
      i32.mul
      set_local $l2
      get_local $p11
      i32.load16_s offset=112
      set_local $p11
      get_local $p10
      set_local $p9
      loop $L10
        get_local $p0
        get_local $p9
        get_local $l2
        i32.add
        i32.add
        get_local $p1
        get_local $p9
        get_local $l0
        i32.add
        i32.add
        i32.load8_u
        get_local $p11
        i32.add
        tee_local $p5
        i32.const -1
        i32.const 0
        get_local $p5
        i32.const 0
        i32.gt_s
        select
        get_local $p5
        i32.const 256
        i32.lt_u
        select
        i32.store8
        get_local $p9
        i32.const 1
        i32.add
        tee_local $p9
        get_local $p6
        i32.ne
        br_if $L10
      end
    end
    get_local $p0
    get_local $p1
    get_local $p2
    get_local $p3
    get_local $p4
    get_local $p6
    get_local $p7
    get_local $p8
    get_local $p10
    get_local $l1
    call $sao_edge_filter_8)
  (func $sao_band_filter_0_8 (type $t17) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (param $p8 i32)
    (local $l0 i32) (local $l1 i32)
    get_global $g0
    i32.const 128
    i32.sub
    tee_local $l0
    set_global $g0
    i32.const 0
    set_local $p5
    get_local $l0
    i32.const 0
    i32.const 128
    call $memset
    set_local $l0
    get_local $p4
    get_local $p8
    i32.add
    i32.load8_u offset=96
    set_local $l1
    get_local $p4
    get_local $p8
    i32.const 10
    i32.mul
    i32.add
    set_local $p4
    loop $L0
      get_local $l0
      get_local $p5
      get_local $l1
      i32.add
      i32.const 31
      i32.and
      i32.const 2
      i32.shl
      i32.add
      get_local $p4
      get_local $p5
      i32.const 1
      i32.add
      tee_local $p5
      i32.const 1
      i32.shl
      i32.add
      i32.load16_s offset=112
      i32.store
      get_local $p5
      i32.const 4
      i32.ne
      br_if $L0
    end
    get_local $p7
    i32.const 1
    i32.ge_s
    if $I1
      get_local $p6
      i32.const 1
      i32.lt_s
      set_local $l1
      i32.const 0
      set_local $p4
      loop $L2
        i32.const 0
        set_local $p5
        get_local $l1
        i32.eqz
        if $I3
          loop $L4
            get_local $p0
            get_local $p5
            i32.add
            get_local $l0
            get_local $p1
            get_local $p5
            i32.add
            i32.load8_u
            tee_local $p8
            i32.const 1
            i32.shr_u
            i32.const 124
            i32.and
            i32.add
            i32.load
            get_local $p8
            i32.add
            tee_local $p8
            i32.const -1
            i32.const 0
            get_local $p8
            i32.const 0
            i32.gt_s
            select
            get_local $p8
            i32.const 256
            i32.lt_u
            select
            i32.store8
            get_local $p5
            i32.const 1
            i32.add
            tee_local $p5
            get_local $p6
            i32.ne
            br_if $L4
          end
        end
        get_local $p1
        get_local $p3
        i32.add
        set_local $p1
        get_local $p0
        get_local $p2
        i32.add
        set_local $p0
        get_local $p4
        i32.const 1
        i32.add
        tee_local $p4
        get_local $p7
        i32.ne
        br_if $L2
      end
    end
    get_local $l0
    i32.const 128
    i32.add
    set_global $g0)
  (func $idct_4x4_dc_8 (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p0
    i32.load16_s
    i32.const 1
    i32.add
    i32.const 1
    i32.shr_s
    i32.const 32
    i32.add
    i32.const 6
    i32.shr_s
    set_local $l2
    loop $L0
      get_local $l0
      i32.const 2
      i32.shl
      set_local $l3
      i32.const 0
      set_local $l1
      loop $L1
        get_local $p0
        get_local $l1
        get_local $l3
        i32.add
        i32.const 1
        i32.shl
        i32.add
        get_local $l2
        i32.store16
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 4
        i32.ne
        br_if $L1
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 4
      i32.ne
      br_if $L0
    end)
  (func $idct_4x4_8 (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32)
    get_local $p0
    set_local $p1
    loop $L0
      get_local $p1
      get_local $p1
      i32.load16_s offset=16
      i32.const 6
      i32.shl
      tee_local $l2
      get_local $p1
      i32.load16_s
      i32.const 6
      i32.shl
      tee_local $l0
      i32.add
      tee_local $l3
      get_local $p1
      i32.load16_s offset=24
      tee_local $l4
      i32.const 36
      i32.mul
      get_local $p1
      i32.load16_s offset=8
      tee_local $l5
      i32.const 83
      i32.mul
      i32.add
      tee_local $l6
      i32.sub
      i32.const -64
      i32.sub
      tee_local $l8
      i32.const 7
      i32.shr_s
      tee_local $l9
      get_local $l8
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l9
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16 offset=24
      get_local $p1
      get_local $l0
      get_local $l2
      i32.sub
      tee_local $l2
      get_local $l4
      i32.const -83
      i32.mul
      get_local $l5
      i32.const 36
      i32.mul
      i32.add
      tee_local $l0
      i32.sub
      i32.const -64
      i32.sub
      tee_local $l4
      i32.const 7
      i32.shr_s
      tee_local $l5
      get_local $l4
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l5
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16 offset=16
      get_local $p1
      get_local $l0
      get_local $l2
      i32.add
      i32.const -64
      i32.sub
      tee_local $l2
      i32.const 7
      i32.shr_s
      tee_local $l0
      get_local $l2
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l0
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16 offset=8
      get_local $p1
      get_local $l3
      get_local $l6
      i32.add
      i32.const -64
      i32.sub
      tee_local $l2
      i32.const 7
      i32.shr_s
      tee_local $l0
      get_local $l2
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l0
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16
      get_local $p1
      i32.const 2
      i32.add
      set_local $p1
      get_local $l1
      i32.const 1
      i32.add
      tee_local $l1
      i32.const 4
      i32.ne
      br_if $L0
    end
    loop $L1
      get_local $p0
      get_local $p0
      i32.load16_s offset=4
      i32.const 6
      i32.shl
      tee_local $p1
      get_local $p0
      i32.load16_s
      i32.const 6
      i32.shl
      tee_local $l1
      i32.add
      tee_local $l2
      get_local $p0
      i32.load16_s offset=6
      tee_local $l0
      i32.const 36
      i32.mul
      get_local $p0
      i32.load16_s offset=2
      tee_local $l3
      i32.const 83
      i32.mul
      i32.add
      tee_local $l4
      i32.sub
      i32.const 2048
      i32.add
      tee_local $l5
      i32.const 12
      i32.shr_s
      tee_local $l6
      get_local $l5
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l6
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16 offset=6
      get_local $p0
      get_local $l1
      get_local $p1
      i32.sub
      tee_local $p1
      get_local $l0
      i32.const -83
      i32.mul
      get_local $l3
      i32.const 36
      i32.mul
      i32.add
      tee_local $l1
      i32.sub
      i32.const 2048
      i32.add
      tee_local $l0
      i32.const 12
      i32.shr_s
      tee_local $l3
      get_local $l0
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l3
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16 offset=4
      get_local $p0
      get_local $p1
      get_local $l1
      i32.add
      i32.const 2048
      i32.add
      tee_local $p1
      i32.const 12
      i32.shr_s
      tee_local $l1
      get_local $p1
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l1
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16 offset=2
      get_local $p0
      get_local $l2
      get_local $l4
      i32.add
      i32.const 2048
      i32.add
      tee_local $p1
      i32.const 12
      i32.shr_s
      tee_local $l1
      get_local $p1
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l1
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16
      get_local $p0
      i32.const 8
      i32.add
      set_local $p0
      get_local $l7
      i32.const 1
      i32.add
      tee_local $l7
      i32.const 4
      i32.ne
      br_if $L1
    end)
  (func $transform_4x4_luma_8 (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32)
    get_local $p0
    set_local $l0
    loop $L0
      get_local $l0
      get_local $l0
      i32.load16_s offset=24
      tee_local $l1
      get_local $l0
      i32.load16_s
      tee_local $l2
      get_local $l0
      i32.load16_s offset=16
      tee_local $l4
      i32.sub
      i32.add
      i32.const 74
      i32.mul
      i32.const -64
      i32.sub
      tee_local $l5
      i32.const 7
      i32.shr_s
      tee_local $l6
      get_local $l5
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l6
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16 offset=16
      get_local $l0
      get_local $l0
      i32.load16_s offset=8
      i32.const 74
      i32.mul
      tee_local $l5
      i32.const -64
      i32.sub
      tee_local $l6
      get_local $l2
      get_local $l1
      i32.sub
      tee_local $l8
      i32.const 55
      i32.mul
      i32.add
      get_local $l1
      get_local $l4
      i32.add
      tee_local $l1
      i32.const -29
      i32.mul
      i32.add
      tee_local $l9
      i32.const 7
      i32.shr_s
      tee_local $l10
      get_local $l9
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l10
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16 offset=8
      get_local $l0
      get_local $l6
      get_local $l2
      get_local $l4
      i32.add
      tee_local $l2
      i32.const 29
      i32.mul
      i32.add
      get_local $l1
      i32.const 55
      i32.mul
      i32.add
      tee_local $l1
      i32.const 7
      i32.shr_s
      tee_local $l4
      get_local $l1
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l4
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16
      get_local $l0
      get_local $l2
      i32.const 55
      i32.mul
      get_local $l8
      i32.const 29
      i32.mul
      i32.add
      get_local $l5
      i32.sub
      i32.const -64
      i32.sub
      tee_local $l1
      i32.const 7
      i32.shr_s
      tee_local $l2
      get_local $l1
      i32.const 31
      i32.shr_s
      i32.const 32767
      i32.xor
      get_local $l2
      i32.const 32768
      i32.add
      i32.const 65536
      i32.lt_u
      select
      i32.store16 offset=24
      get_local $l0
      i32.const 2
      i32.add
      set_local $l0
      get_local $l3
      i32.const 1
      i32.add
      tee_local $l3
      i32.const 4
      i32.ne
      br_if $L0
    end
    loop $L1
      get_local $p0
      get_local $p0
      i32.load16_s offset=6
      tee_local $l0
      get_local $p0
      i32.load16_s
      tee_local $l3
      get_local $p0
      i32.load16_s offset=4
      tee_local $l1
      i32.sub
      i32.add
      i32.const 74
      i32.mul
      i32.const 2048
      i32.add
      i32.const 12
      i32.shr_u
      i32.store16 offset=4
      get_local $p0
      get_local $p0
      i32.load16_s offset=2
      i32.const 74
      i32.mul
      tee_local $l2
      i32.const 2048
      i32.add
      tee_local $l4
      get_local $l3
      get_local $l0
      i32.sub
      tee_local $l5
      i32.const 55
      i32.mul
      i32.add
      get_local $l0
      get_local $l1
      i32.add
      tee_local $l0
      i32.const -29
      i32.mul
      i32.add
      i32.const 12
      i32.shr_u
      i32.store16 offset=2
      get_local $p0
      get_local $l4
      get_local $l1
      get_local $l3
      i32.add
      tee_local $l3
      i32.const 29
      i32.mul
      i32.add
      get_local $l0
      i32.const 55
      i32.mul
      i32.add
      i32.const 12
      i32.shr_u
      i32.store16
      get_local $p0
      get_local $l3
      i32.const 55
      i32.mul
      get_local $l5
      i32.const 29
      i32.mul
      i32.add
      get_local $l2
      i32.sub
      i32.const 2048
      i32.add
      i32.const 12
      i32.shr_u
      i32.store16 offset=6
      get_local $p0
      i32.const 8
      i32.add
      set_local $p0
      get_local $l7
      i32.const 1
      i32.add
      tee_local $l7
      i32.const 4
      i32.ne
      br_if $L1
    end)
  (func $transform_rdpcm_8 (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32)
    i32.const 1
    get_local $p1
    i32.shl
    set_local $l0
    block $B0
      get_local $p2
      if $I1
        get_local $l0
        i32.const 2
        i32.lt_s
        br_if $B0
        get_local $l0
        i32.const 1
        i32.sub
        set_local $l2
        get_local $l0
        i32.const 1
        i32.shl
        set_local $l3
        loop $L2
          get_local $p0
          get_local $l3
          i32.add
          set_local $p2
          i32.const 0
          set_local $p1
          loop $L3
            get_local $p2
            get_local $p1
            i32.const 1
            i32.shl
            tee_local $l4
            i32.add
            tee_local $l5
            get_local $l5
            i32.load16_u
            get_local $p0
            get_local $l4
            i32.add
            i32.load16_u
            i32.add
            i32.store16
            get_local $p1
            i32.const 1
            i32.add
            tee_local $p1
            get_local $l0
            i32.ne
            br_if $L3
          end
          get_local $p2
          set_local $p0
          get_local $l1
          i32.const 1
          i32.add
          tee_local $l1
          get_local $l2
          i32.ne
          br_if $L2
        end
        br $B0
      end
      get_local $p1
      i32.const 31
      i32.eq
      br_if $B0
      get_local $l0
      i32.const 1
      get_local $l0
      i32.const 1
      i32.gt_s
      select
      set_local $l2
      get_local $l0
      i32.const 2
      i32.lt_s
      set_local $l3
      loop $L4
        get_local $l3
        i32.eqz
        if $I5
          get_local $p0
          i32.load16_u
          set_local $p2
          i32.const 1
          set_local $p1
          loop $L6
            get_local $p0
            get_local $p1
            i32.const 1
            i32.shl
            i32.add
            tee_local $l4
            get_local $l4
            i32.load16_u
            get_local $p2
            i32.add
            tee_local $p2
            i32.store16
            get_local $p1
            i32.const 1
            i32.add
            tee_local $p1
            get_local $l0
            i32.ne
            br_if $L6
          end
        end
        get_local $p0
        get_local $l0
        i32.const 1
        i32.shl
        i32.add
        set_local $p0
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        get_local $l2
        i32.ne
        br_if $L4
      end
    end)
  (func $transform_skip_8 (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    i32.const 1
    get_local $p1
    i32.shl
    set_local $l0
    block $B0
      get_local $p1
      i32.const 7
      i32.ge_s
      if $I1
        get_local $p1
        i32.const 31
        i32.eq
        br_if $B0
        get_local $p1
        i32.const 7
        i32.sub
        set_local $l2
        get_local $l0
        i32.const 1
        get_local $l0
        i32.const 1
        i32.gt_s
        select
        set_local $l0
        loop $L2
          i32.const 0
          set_local $p1
          loop $L3
            get_local $p0
            get_local $p0
            i32.load16_s
            get_local $l2
            i32.shl
            i32.store16
            get_local $p0
            i32.const 2
            i32.add
            set_local $p0
            get_local $p1
            i32.const 1
            i32.add
            tee_local $p1
            get_local $l0
            i32.ne
            br_if $L3
          end
          get_local $l1
          i32.const 1
          i32.add
          tee_local $l1
          get_local $l0
          i32.ne
          br_if $L2
        end
        br $B0
      end
      i32.const 7
      get_local $p1
      i32.sub
      set_local $l2
      get_local $l0
      i32.const 1
      get_local $l0
      i32.const 1
      i32.gt_s
      select
      set_local $l0
      i32.const 1
      i32.const 6
      get_local $p1
      i32.sub
      i32.shl
      set_local $l3
      loop $L4
        i32.const 0
        set_local $p1
        loop $L5
          get_local $p0
          get_local $l3
          get_local $p0
          i32.load16_s
          i32.add
          get_local $l2
          i32.shr_s
          i32.store16
          get_local $p0
          i32.const 2
          i32.add
          set_local $p0
          get_local $p1
          i32.const 1
          i32.add
          tee_local $p1
          get_local $l0
          i32.ne
          br_if $L5
        end
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        get_local $l0
        i32.ne
        br_if $L4
      end
    end)
  (func $transform_add4x4_8 (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    loop $L0
      i32.const 0
      set_local $l1
      loop $L1
        get_local $p0
        get_local $l1
        i32.add
        tee_local $l0
        get_local $p1
        i32.load16_s
        get_local $l0
        i32.load8_u
        i32.add
        tee_local $l0
        i32.const -1
        i32.const 0
        get_local $l0
        i32.const 0
        i32.gt_s
        select
        get_local $l0
        i32.const 256
        i32.lt_u
        select
        i32.store8
        get_local $p1
        i32.const 2
        i32.add
        set_local $p1
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 4
        i32.ne
        br_if $L1
      end
      get_local $p0
      get_local $p2
      i32.add
      set_local $p0
      get_local $l2
      i32.const 1
      i32.add
      tee_local $l2
      i32.const 4
      i32.ne
      br_if $L0
    end)
  (func $put_pcm_8 (type $t6) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p3
    i32.const 1
    i32.ge_s
    if $I0
      i32.const 8
      get_local $p5
      i32.sub
      set_local $l2
      get_local $p2
      i32.const 1
      i32.lt_s
      set_local $l3
      loop $L1
        i32.const 0
        set_local $l0
        get_local $l3
        i32.eqz
        if $I2
          loop $L3
            get_local $p0
            get_local $l0
            i32.add
            get_local $p4
            get_local $p5
            call $get_bits
            get_local $l2
            i32.shl
            i32.store8
            get_local $l0
            i32.const 1
            i32.add
            tee_local $l0
            get_local $p2
            i32.ne
            br_if $L3
          end
        end
        get_local $p0
        get_local $p1
        i32.add
        set_local $p0
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        get_local $p3
        i32.ne
        br_if $L1
      end
    end)
  (func $sao_edge_filter_1_8 (type $t15) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (param $p8 i32) (param $p9 i32) (param $p10 i32) (param $p11 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32)
    get_local $p4
    get_local $p8
    i32.const 10
    i32.mul
    i32.add
    set_local $l3
    block $B0
      block $B1
        get_local $p4
        get_local $p8
        i32.const 2
        i32.shl
        i32.add
        i32.load offset=100
        tee_local $l5
        i32.const 1
        i32.eq
        if $I2
          br $B1
        end
        block $B3
          get_local $p5
          i32.load
          i32.eqz
          if $I4
            br $B3
          end
          i32.const 1
          set_local $l1
          get_local $p7
          i32.const 1
          i32.lt_s
          br_if $B3
          get_local $l3
          i32.load16_s offset=112
          set_local $l2
          loop $L5
            get_local $p0
            get_local $p2
            get_local $l0
            i32.mul
            i32.add
            get_local $p1
            get_local $p3
            get_local $l0
            i32.mul
            i32.add
            i32.load8_u
            get_local $l2
            i32.add
            tee_local $l1
            i32.const -1
            i32.const 0
            get_local $l1
            i32.const 0
            i32.gt_s
            select
            get_local $l1
            i32.const 256
            i32.lt_u
            select
            i32.store8
            i32.const 1
            set_local $l1
            get_local $l0
            i32.const 1
            i32.add
            tee_local $l0
            get_local $p7
            i32.ne
            br_if $L5
          end
        end
        block $B6
          get_local $p5
          i32.load offset=8
          i32.eqz
          br_if $B6
          get_local $p6
          i32.const 1
          i32.sub
          set_local $p6
          get_local $p7
          i32.const 1
          i32.lt_s
          br_if $B6
          get_local $l3
          i32.load16_s offset=112
          set_local $l2
          i32.const 0
          set_local $l0
          loop $L7
            get_local $p0
            get_local $p2
            get_local $l0
            i32.mul
            get_local $p6
            i32.add
            i32.add
            get_local $p1
            get_local $p3
            get_local $l0
            i32.mul
            get_local $p6
            i32.add
            i32.add
            i32.load8_u
            get_local $l2
            i32.add
            tee_local $l4
            i32.const -1
            i32.const 0
            get_local $l4
            i32.const 0
            i32.gt_s
            select
            get_local $l4
            i32.const 256
            i32.lt_u
            select
            i32.store8
            get_local $l0
            i32.const 1
            i32.add
            tee_local $l0
            get_local $p7
            i32.ne
            br_if $L7
          end
        end
        get_local $l5
        br_if $B1
        br $B0
      end
      block $B8
        get_local $p5
        i32.load offset=4
        i32.eqz
        br_if $B8
        i32.const 1
        set_local $l7
        get_local $p6
        get_local $l1
        i32.le_s
        br_if $B8
        get_local $l3
        i32.load16_s offset=112
        set_local $l2
        get_local $l1
        set_local $l0
        loop $L9
          get_local $p0
          get_local $l0
          i32.add
          get_local $p1
          get_local $l0
          i32.add
          i32.load8_u
          get_local $l2
          i32.add
          tee_local $l4
          i32.const -1
          i32.const 0
          get_local $l4
          i32.const 0
          i32.gt_s
          select
          get_local $l4
          i32.const 256
          i32.lt_u
          select
          i32.store8
          get_local $l0
          i32.const 1
          i32.add
          tee_local $l0
          get_local $p6
          i32.ne
          br_if $L9
        end
      end
      i32.const 1
      set_local $l6
      get_local $p5
      i32.load offset=12
      i32.eqz
      br_if $B0
      get_local $p7
      i32.const 1
      i32.sub
      set_local $p7
      get_local $p6
      get_local $l1
      i32.le_s
      br_if $B0
      get_local $p3
      get_local $p7
      i32.mul
      set_local $l8
      get_local $p2
      get_local $p7
      i32.mul
      set_local $l4
      get_local $l3
      i32.load16_s offset=112
      set_local $l2
      get_local $l1
      set_local $l0
      loop $L10
        get_local $p0
        get_local $l0
        get_local $l4
        i32.add
        i32.add
        get_local $p1
        get_local $l0
        get_local $l8
        i32.add
        i32.add
        i32.load8_u
        get_local $l2
        i32.add
        tee_local $l6
        i32.const -1
        i32.const 0
        get_local $l6
        i32.const 0
        i32.gt_s
        select
        get_local $l6
        i32.const 256
        i32.lt_u
        select
        i32.store8
        i32.const 1
        set_local $l6
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p6
        i32.ne
        br_if $L10
      end
    end
    get_local $p0
    get_local $p1
    get_local $p2
    get_local $p3
    get_local $p4
    get_local $p6
    get_local $p7
    get_local $p8
    get_local $l1
    get_local $l7
    call $sao_edge_filter_8
    block $B11 (result i32)
      block $B12
        block $B13
          get_local $l5
          i32.const 2
          i32.ne
          br_if $B13
          get_local $p11
          i32.load8_u
          br_if $B13
          i32.const 0
          set_local $l3
          i32.const 0
          set_local $p4
          get_local $p5
          i32.load
          i32.eqz
          if $I14
            get_local $p5
            i32.load offset=4
            i32.eqz
            set_local $p4
          end
          get_local $p11
          i32.const 1
          i32.add
          set_local $p8
          br $B12
        end
        get_local $l5
        i32.const 3
        i32.eq
        set_local $l3
        get_local $p11
        i32.const 1
        i32.add
        set_local $p8
        i32.const 0
        set_local $p4
        get_local $l5
        i32.const 3
        i32.ne
        br_if $B12
        get_local $p11
        i32.load8_u offset=1
        br_if $B12
        i32.const 1
        set_local $l3
        get_local $p5
        i32.load offset=4
        br_if $B12
        get_local $p5
        i32.load offset=8
        i32.eqz
        br $B11
      end
      i32.const 0
    end
    set_local $l4
    i32.const 0
    set_local $l2
    block $B15 (result i32)
      i32.const 0
      get_local $l5
      i32.const 2
      i32.ne
      br_if $B15
      drop
      i32.const 0
      get_local $p11
      i32.load8_u offset=2
      br_if $B15
      drop
      i32.const 0
      get_local $p5
      i32.load offset=8
      br_if $B15
      drop
      get_local $p5
      i32.load offset=12
      i32.eqz
    end
    set_local $l8
    block $B16
      get_local $p11
      i32.load8_u offset=3
      get_local $l3
      i32.const 1
      i32.xor
      i32.or
      br_if $B16
      get_local $p5
      i32.load
      br_if $B16
      get_local $p5
      i32.load offset=12
      i32.eqz
      set_local $l2
    end
    block $B17
      get_local $l5
      i32.const 1
      i32.eq
      br_if $B17
      get_local $p9
      i32.load8_u
      i32.eqz
      br_if $B17
      get_local $p4
      get_local $l7
      i32.add
      tee_local $l0
      get_local $p7
      get_local $l2
      i32.sub
      tee_local $p5
      i32.ge_s
      br_if $B17
      loop $L18
        get_local $p0
        get_local $p2
        get_local $l0
        i32.mul
        i32.add
        get_local $p1
        get_local $p3
        get_local $l0
        i32.mul
        i32.add
        i32.load8_u
        i32.store8
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p5
        i32.lt_s
        br_if $L18
      end
    end
    block $B19
      get_local $l5
      i32.const 1
      i32.eq
      br_if $B19
      get_local $p9
      i32.load8_u offset=1
      i32.eqz
      br_if $B19
      get_local $l4
      get_local $l7
      i32.add
      tee_local $l0
      get_local $p7
      get_local $l8
      i32.sub
      tee_local $p5
      i32.ge_s
      br_if $B19
      get_local $p6
      i32.const 1
      i32.sub
      set_local $p9
      loop $L20
        get_local $p0
        get_local $p9
        get_local $p2
        get_local $l0
        i32.mul
        i32.add
        i32.add
        get_local $p1
        get_local $p9
        get_local $p3
        get_local $l0
        i32.mul
        i32.add
        i32.add
        i32.load8_u
        i32.store8
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p5
        i32.lt_s
        br_if $L20
      end
    end
    block $B21
      get_local $p10
      i32.load8_u
      i32.eqz
      get_local $l6
      i32.const 1
      i32.xor
      i32.or
      br_if $B21
      get_local $p4
      get_local $l1
      i32.add
      tee_local $l0
      get_local $p6
      get_local $l4
      i32.sub
      tee_local $p4
      i32.ge_s
      br_if $B21
      loop $L22
        get_local $p0
        get_local $l0
        i32.add
        get_local $p1
        get_local $l0
        i32.add
        i32.load8_u
        i32.store8
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p4
        i32.lt_s
        br_if $L22
      end
    end
    block $B23
      get_local $p10
      i32.load8_u offset=1
      i32.eqz
      get_local $l6
      i32.const 1
      i32.xor
      i32.or
      br_if $B23
      get_local $l1
      get_local $l2
      i32.add
      tee_local $l0
      get_local $p6
      get_local $l8
      i32.sub
      tee_local $p9
      i32.ge_s
      br_if $B23
      get_local $p7
      i32.const 1
      i32.sub
      tee_local $p4
      get_local $p2
      i32.mul
      set_local $p5
      get_local $p3
      get_local $p4
      i32.mul
      set_local $p4
      loop $L24
        get_local $p0
        get_local $p5
        get_local $l0
        i32.add
        i32.add
        get_local $p1
        get_local $p4
        get_local $l0
        i32.add
        i32.add
        i32.load8_u
        i32.store8
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p9
        i32.lt_s
        br_if $L24
      end
    end
    block $B25
      get_local $l5
      i32.const 2
      i32.ne
      br_if $B25
      get_local $p11
      i32.load8_u
      i32.eqz
      br_if $B25
      get_local $p0
      get_local $p1
      i32.load8_u
      i32.store8
    end
    get_local $p8
    i32.load8_u
    i32.eqz
    get_local $l3
    i32.const 1
    i32.xor
    i32.or
    i32.eqz
    if $I26
      get_local $p0
      get_local $p6
      i32.const 1
      i32.sub
      tee_local $p4
      i32.add
      get_local $p1
      get_local $p4
      i32.add
      i32.load8_u
      i32.store8
    end
    block $B27
      get_local $l5
      i32.const 2
      i32.ne
      br_if $B27
      get_local $p11
      i32.load8_u offset=2
      i32.eqz
      br_if $B27
      get_local $p0
      get_local $p6
      i32.const 1
      i32.sub
      tee_local $p5
      get_local $p7
      i32.const 1
      i32.sub
      tee_local $p4
      get_local $p2
      i32.mul
      i32.add
      i32.add
      get_local $p1
      get_local $p5
      get_local $p3
      get_local $p4
      i32.mul
      i32.add
      i32.add
      i32.load8_u
      i32.store8
    end
    get_local $p11
    i32.load8_u offset=3
    i32.eqz
    get_local $l3
    i32.const 1
    i32.xor
    i32.or
    i32.eqz
    if $I28
      get_local $p0
      get_local $p2
      get_local $p7
      i32.const 1
      i32.sub
      tee_local $p2
      i32.mul
      i32.add
      get_local $p1
      get_local $p2
      get_local $p3
      i32.mul
      i32.add
      i32.load8_u
      i32.store8
    end)
  (func $idct_32x32_dc_8 (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p0
    i32.load16_s
    i32.const 1
    i32.add
    i32.const 1
    i32.shr_s
    i32.const 32
    i32.add
    i32.const 6
    i32.shr_s
    set_local $l2
    loop $L0
      get_local $l0
      i32.const 5
      i32.shl
      set_local $l3
      i32.const 0
      set_local $l1
      loop $L1
        get_local $p0
        get_local $l1
        get_local $l3
        i32.add
        i32.const 1
        i32.shl
        i32.add
        get_local $l2
        i32.store16
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 32
        i32.ne
        br_if $L1
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 32
      i32.ne
      br_if $L0
    end)
  (func $idct_16x16_dc_8 (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p0
    i32.load16_s
    i32.const 1
    i32.add
    i32.const 1
    i32.shr_s
    i32.const 32
    i32.add
    i32.const 6
    i32.shr_s
    set_local $l2
    loop $L0
      get_local $l0
      i32.const 4
      i32.shl
      set_local $l3
      i32.const 0
      set_local $l1
      loop $L1
        get_local $p0
        get_local $l1
        get_local $l3
        i32.add
        i32.const 1
        i32.shl
        i32.add
        get_local $l2
        i32.store16
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 16
        i32.ne
        br_if $L1
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 16
      i32.ne
      br_if $L0
    end)
  (func $idct_8x8_dc_8 (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p0
    i32.load16_s
    i32.const 1
    i32.add
    i32.const 1
    i32.shr_s
    i32.const 32
    i32.add
    i32.const 6
    i32.shr_s
    set_local $l2
    loop $L0
      get_local $l0
      i32.const 3
      i32.shl
      set_local $l3
      i32.const 0
      set_local $l1
      loop $L1
        get_local $p0
        get_local $l1
        get_local $l3
        i32.add
        i32.const 1
        i32.shl
        i32.add
        get_local $l2
        i32.store16
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 8
        i32.ne
        br_if $L1
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 8
      i32.ne
      br_if $L0
    end)
  (func $idct_32x32_8 (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32)
    get_global $g0
    i32.const 208
    i32.sub
    tee_local $l1
    set_global $g0
    get_local $p1
    i32.const 28
    get_local $p1
    i32.const 28
    i32.lt_s
    select
    i32.const 4
    i32.add
    set_local $l6
    get_local $p0
    set_local $l3
    loop $L0
      get_local $l1
      i32.const 80
      i32.add
      i32.const 0
      i32.const 64
      call $memset
      drop
      i32.const 0
      set_local $l4
      loop $L1
        get_local $l6
        i32.const 2
        i32.ge_s
        if $I2
          get_local $l1
          i32.const 80
          i32.add
          get_local $l4
          i32.const 2
          i32.shl
          i32.add
          tee_local $l7
          i32.load
          set_local $l2
          i32.const 1
          set_local $l0
          loop $L3
            get_local $l3
            get_local $l0
            i32.const 6
            i32.shl
            i32.add
            i32.load16_s
            get_local $l0
            i32.const 5
            i32.shl
            get_local $l4
            i32.add
            i32.const 3744
            i32.add
            i32.load8_s
            i32.mul
            get_local $l2
            i32.add
            set_local $l2
            get_local $l0
            i32.const 2
            i32.add
            tee_local $l0
            get_local $l6
            i32.lt_s
            br_if $L3
          end
          get_local $l7
          get_local $l2
          i32.store
        end
        get_local $l4
        i32.const 1
        i32.add
        tee_local $l4
        i32.const 16
        i32.ne
        br_if $L1
      end
      get_local $l1
      i64.const 0
      i64.store offset=40
      get_local $l1
      i64.const 0
      i64.store offset=32
      get_local $l1
      i64.const 0
      i64.store offset=24
      get_local $l1
      i64.const 0
      i64.store offset=16
      get_local $l6
      i32.const 2
      i32.div_s
      set_local $l7
      i32.const 0
      set_local $l4
      loop $L4
        get_local $l6
        i32.const 4
        i32.ge_s
        if $I5
          get_local $l1
          i32.const 16
          i32.add
          get_local $l4
          i32.const 2
          i32.shl
          i32.add
          tee_local $l5
          i32.load
          set_local $l2
          i32.const 1
          set_local $l0
          loop $L6
            get_local $l3
            get_local $l0
            i32.const 7
            i32.shl
            i32.add
            i32.load16_s
            get_local $l0
            i32.const 6
            i32.shl
            get_local $l4
            i32.add
            i32.const 3744
            i32.add
            i32.load8_s
            i32.mul
            get_local $l2
            i32.add
            set_local $l2
            get_local $l0
            i32.const 2
            i32.add
            tee_local $l0
            get_local $l7
            i32.lt_s
            br_if $L6
          end
          get_local $l5
          get_local $l2
          i32.store
        end
        get_local $l4
        i32.const 1
        i32.add
        tee_local $l4
        i32.const 8
        i32.ne
        br_if $L4
      end
      get_local $l1
      i64.const 0
      i64.store offset=8
      get_local $l1
      i64.const 0
      i64.store
      i32.const 0
      set_local $l4
      loop $L7
        get_local $l1
        get_local $l4
        i32.const 2
        i32.shl
        i32.add
        tee_local $l7
        i32.load
        set_local $l2
        i32.const 1
        set_local $l0
        loop $L8
          get_local $l3
          get_local $l0
          i32.const 8
          i32.shl
          i32.add
          i32.load16_s
          get_local $l0
          i32.const 7
          i32.shl
          get_local $l4
          i32.add
          i32.const 3744
          i32.add
          i32.load8_s
          i32.mul
          get_local $l2
          i32.add
          set_local $l2
          get_local $l0
          i32.const 6
          i32.lt_u
          set_local $l5
          get_local $l0
          i32.const 2
          i32.add
          set_local $l0
          get_local $l5
          br_if $L8
        end
        get_local $l7
        get_local $l2
        i32.store
        get_local $l4
        i32.const 1
        i32.add
        tee_local $l4
        i32.const 4
        i32.ne
        br_if $L7
      end
      get_local $l1
      get_local $l3
      i32.load16_s offset=1024
      i32.const 6
      i32.shl
      tee_local $l0
      get_local $l3
      i32.load16_s
      i32.const 6
      i32.shl
      tee_local $l2
      i32.add
      tee_local $l4
      get_local $l3
      i32.load16_s offset=1536
      tee_local $l7
      i32.const 36
      i32.mul
      get_local $l3
      i32.load16_s offset=512
      tee_local $l5
      i32.const 83
      i32.mul
      i32.add
      tee_local $l9
      i32.sub
      i32.store offset=156
      get_local $l1
      get_local $l2
      get_local $l0
      i32.sub
      tee_local $l0
      get_local $l7
      i32.const -83
      i32.mul
      get_local $l5
      i32.const 36
      i32.mul
      i32.add
      tee_local $l2
      i32.sub
      i32.store offset=152
      get_local $l1
      get_local $l0
      get_local $l2
      i32.add
      i32.store offset=148
      get_local $l1
      get_local $l4
      get_local $l9
      i32.add
      tee_local $l0
      i32.store offset=144
      get_local $l1
      get_local $l0
      get_local $l1
      i32.load
      tee_local $l2
      i32.sub
      i32.store offset=76
      get_local $l1
      get_local $l0
      get_local $l2
      i32.add
      i32.store offset=48
      i32.const 1
      set_local $l0
      loop $L9
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $l2
        get_local $l1
        i32.const 48
        i32.add
        i32.add
        get_local $l1
        get_local $l2
        i32.add
        i32.load
        tee_local $l4
        get_local $l1
        i32.const 144
        i32.add
        get_local $l2
        i32.add
        i32.load
        tee_local $l2
        i32.add
        i32.store
        i32.const 0
        get_local $l0
        i32.sub
        i32.const 2
        i32.shl
        get_local $l1
        i32.add
        get_local $l2
        get_local $l4
        i32.sub
        i32.store offset=76
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 4
        i32.ne
        br_if $L9
      end
      i32.const 0
      set_local $l0
      loop $L10
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $l2
        get_local $l1
        i32.const 144
        i32.add
        i32.add
        get_local $l1
        i32.const 16
        i32.add
        get_local $l2
        i32.add
        i32.load
        tee_local $l4
        get_local $l1
        i32.const 48
        i32.add
        get_local $l2
        i32.add
        i32.load
        tee_local $l2
        i32.add
        i32.store
        i32.const 0
        get_local $l0
        i32.sub
        i32.const 2
        i32.shl
        get_local $l1
        i32.add
        get_local $l2
        get_local $l4
        i32.sub
        i32.store offset=204
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 8
        i32.ne
        br_if $L10
      end
      i32.const 0
      set_local $l0
      loop $L11
        get_local $l3
        get_local $l0
        i32.const 6
        i32.shl
        tee_local $l2
        i32.add
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $l4
        get_local $l1
        i32.const 144
        i32.add
        i32.add
        i32.load
        tee_local $l7
        get_local $l1
        i32.const 80
        i32.add
        get_local $l4
        i32.add
        i32.load
        tee_local $l4
        i32.add
        i32.const -64
        i32.sub
        tee_local $l5
        i32.const 7
        i32.shr_s
        tee_local $l9
        get_local $l5
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l9
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $l3
        get_local $l2
        i32.sub
        i32.const 1984
        i32.add
        get_local $l7
        get_local $l4
        i32.sub
        i32.const -64
        i32.sub
        tee_local $l2
        i32.const 7
        i32.shr_s
        tee_local $l4
        get_local $l2
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l4
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 16
        i32.ne
        br_if $L11
      end
      get_local $l6
      get_local $l6
      i32.const 4
      i32.sub
      get_local $l8
      i32.const 3
      i32.and
      select
      get_local $l6
      get_local $l8
      select
      get_local $l6
      get_local $l6
      i32.const 31
      i32.le_s
      select
      set_local $l6
      get_local $l3
      i32.const 2
      i32.add
      set_local $l3
      get_local $l8
      i32.const 1
      i32.add
      tee_local $l8
      i32.const 32
      i32.ne
      br_if $L0
    end
    get_local $p1
    i32.const 32
    get_local $p1
    i32.const 32
    i32.lt_s
    select
    tee_local $p1
    i32.const 2
    i32.div_s
    set_local $l6
    get_local $p1
    i32.const 4
    i32.lt_s
    set_local $l7
    i32.const 0
    set_local $l8
    loop $L12
      get_local $l1
      i32.const 80
      i32.add
      i32.const 0
      i32.const 64
      call $memset
      drop
      i32.const 0
      set_local $l2
      loop $L13
        get_local $p1
        i32.const 2
        i32.ge_s
        if $I14
          get_local $l1
          i32.const 80
          i32.add
          get_local $l2
          i32.const 2
          i32.shl
          i32.add
          tee_local $l4
          i32.load
          set_local $l3
          i32.const 1
          set_local $l0
          loop $L15
            get_local $p0
            get_local $l0
            i32.const 1
            i32.shl
            i32.add
            i32.load16_s
            get_local $l0
            i32.const 5
            i32.shl
            get_local $l2
            i32.add
            i32.const 3744
            i32.add
            i32.load8_s
            i32.mul
            get_local $l3
            i32.add
            set_local $l3
            get_local $l0
            i32.const 2
            i32.add
            tee_local $l0
            get_local $p1
            i32.lt_s
            br_if $L15
          end
          get_local $l4
          get_local $l3
          i32.store
        end
        get_local $l2
        i32.const 1
        i32.add
        tee_local $l2
        i32.const 16
        i32.ne
        br_if $L13
      end
      get_local $l1
      i64.const 0
      i64.store offset=40
      get_local $l1
      i64.const 0
      i64.store offset=32
      get_local $l1
      i64.const 0
      i64.store offset=24
      get_local $l1
      i64.const 0
      i64.store offset=16
      i32.const 0
      set_local $l2
      loop $L16
        get_local $l7
        i32.eqz
        if $I17
          get_local $l1
          i32.const 16
          i32.add
          get_local $l2
          i32.const 2
          i32.shl
          i32.add
          tee_local $l4
          i32.load
          set_local $l3
          i32.const 1
          set_local $l0
          loop $L18
            get_local $p0
            get_local $l0
            i32.const 2
            i32.shl
            i32.add
            i32.load16_s
            get_local $l0
            i32.const 6
            i32.shl
            get_local $l2
            i32.add
            i32.const 3744
            i32.add
            i32.load8_s
            i32.mul
            get_local $l3
            i32.add
            set_local $l3
            get_local $l0
            i32.const 2
            i32.add
            tee_local $l0
            get_local $l6
            i32.lt_s
            br_if $L18
          end
          get_local $l4
          get_local $l3
          i32.store
        end
        get_local $l2
        i32.const 1
        i32.add
        tee_local $l2
        i32.const 8
        i32.ne
        br_if $L16
      end
      get_local $l1
      i64.const 0
      i64.store offset=8
      get_local $l1
      i64.const 0
      i64.store
      i32.const 0
      set_local $l4
      loop $L19
        get_local $l1
        get_local $l4
        i32.const 2
        i32.shl
        i32.add
        tee_local $l2
        i32.load
        set_local $l3
        i32.const 1
        set_local $l0
        loop $L20
          get_local $p0
          get_local $l0
          i32.const 3
          i32.shl
          i32.add
          i32.load16_s
          get_local $l0
          i32.const 7
          i32.shl
          get_local $l4
          i32.add
          i32.const 3744
          i32.add
          i32.load8_s
          i32.mul
          get_local $l3
          i32.add
          set_local $l3
          get_local $l0
          i32.const 6
          i32.lt_u
          set_local $l5
          get_local $l0
          i32.const 2
          i32.add
          set_local $l0
          get_local $l5
          br_if $L20
        end
        get_local $l2
        get_local $l3
        i32.store
        get_local $l4
        i32.const 1
        i32.add
        tee_local $l4
        i32.const 4
        i32.ne
        br_if $L19
      end
      get_local $l1
      get_local $p0
      i32.load16_s offset=32
      i32.const 6
      i32.shl
      tee_local $l3
      get_local $p0
      i32.load16_s
      i32.const 6
      i32.shl
      tee_local $l0
      i32.add
      tee_local $l2
      get_local $p0
      i32.load16_s offset=48
      tee_local $l4
      i32.const 36
      i32.mul
      get_local $p0
      i32.load16_s offset=16
      tee_local $l5
      i32.const 83
      i32.mul
      i32.add
      tee_local $l9
      i32.sub
      i32.store offset=156
      get_local $l1
      get_local $l0
      get_local $l3
      i32.sub
      tee_local $l3
      get_local $l4
      i32.const -83
      i32.mul
      get_local $l5
      i32.const 36
      i32.mul
      i32.add
      tee_local $l0
      i32.sub
      i32.store offset=152
      get_local $l1
      get_local $l0
      get_local $l3
      i32.add
      i32.store offset=148
      get_local $l1
      get_local $l2
      get_local $l9
      i32.add
      tee_local $l3
      i32.store offset=144
      get_local $l1
      get_local $l3
      get_local $l1
      i32.load
      tee_local $l0
      i32.sub
      i32.store offset=76
      get_local $l1
      get_local $l0
      get_local $l3
      i32.add
      i32.store offset=48
      i32.const 1
      set_local $l0
      loop $L21
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $l3
        get_local $l1
        i32.const 48
        i32.add
        i32.add
        get_local $l1
        get_local $l3
        i32.add
        i32.load
        tee_local $l2
        get_local $l1
        i32.const 144
        i32.add
        get_local $l3
        i32.add
        i32.load
        tee_local $l3
        i32.add
        i32.store
        i32.const 0
        get_local $l0
        i32.sub
        i32.const 2
        i32.shl
        get_local $l1
        i32.add
        get_local $l3
        get_local $l2
        i32.sub
        i32.store offset=76
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 4
        i32.ne
        br_if $L21
      end
      i32.const 0
      set_local $l0
      loop $L22
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $l3
        get_local $l1
        i32.const 144
        i32.add
        i32.add
        get_local $l1
        i32.const 16
        i32.add
        get_local $l3
        i32.add
        i32.load
        tee_local $l2
        get_local $l1
        i32.const 48
        i32.add
        get_local $l3
        i32.add
        i32.load
        tee_local $l3
        i32.add
        i32.store
        i32.const 0
        get_local $l0
        i32.sub
        i32.const 2
        i32.shl
        get_local $l1
        i32.add
        get_local $l3
        get_local $l2
        i32.sub
        i32.store offset=204
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 8
        i32.ne
        br_if $L22
      end
      i32.const 0
      set_local $l0
      loop $L23
        get_local $p0
        get_local $l0
        i32.const 1
        i32.shl
        i32.add
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $l3
        get_local $l1
        i32.const 144
        i32.add
        i32.add
        i32.load
        tee_local $l2
        get_local $l1
        i32.const 80
        i32.add
        get_local $l3
        i32.add
        i32.load
        tee_local $l3
        i32.add
        i32.const 2048
        i32.add
        tee_local $l4
        i32.const 12
        i32.shr_s
        tee_local $l5
        get_local $l4
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l5
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $p0
        i32.const 31
        get_local $l0
        i32.sub
        i32.const 1
        i32.shl
        i32.add
        get_local $l2
        get_local $l3
        i32.sub
        i32.const 2048
        i32.add
        tee_local $l3
        i32.const 12
        i32.shr_s
        tee_local $l2
        get_local $l3
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l2
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 16
        i32.ne
        br_if $L23
      end
      get_local $p0
      i32.const -64
      i32.sub
      set_local $p0
      get_local $l8
      i32.const 1
      i32.add
      tee_local $l8
      i32.const 32
      i32.ne
      br_if $L12
    end
    get_local $l1
    i32.const 208
    i32.add
    set_global $g0)
  (func $idct_16x16_8 (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32)
    get_local $p1
    i32.const 12
    get_local $p1
    i32.const 12
    i32.lt_s
    select
    i32.const 4
    i32.add
    set_local $l5
    get_global $g0
    i32.const 96
    i32.sub
    tee_local $l1
    set_local $l9
    get_local $p0
    set_local $l4
    loop $L0
      get_local $l9
      i64.const 0
      i64.store offset=56
      get_local $l1
      i64.const 0
      i64.store offset=48
      get_local $l1
      i64.const 0
      i64.store offset=40
      get_local $l1
      i64.const 0
      i64.store offset=32
      i32.const 0
      set_local $l3
      loop $L1
        get_local $l5
        i32.const 2
        i32.ge_s
        if $I2
          get_local $l1
          i32.const 32
          i32.add
          get_local $l3
          i32.const 2
          i32.shl
          i32.add
          tee_local $l6
          i32.load
          set_local $l2
          i32.const 1
          set_local $l0
          loop $L3
            get_local $l4
            get_local $l0
            i32.const 5
            i32.shl
            i32.add
            i32.load16_s
            get_local $l0
            i32.const 6
            i32.shl
            get_local $l3
            i32.add
            i32.const 3744
            i32.add
            i32.load8_s
            i32.mul
            get_local $l2
            i32.add
            set_local $l2
            get_local $l0
            i32.const 2
            i32.add
            tee_local $l0
            get_local $l5
            i32.lt_s
            br_if $L3
          end
          get_local $l6
          get_local $l2
          i32.store
        end
        get_local $l3
        i32.const 1
        i32.add
        tee_local $l3
        i32.const 8
        i32.ne
        br_if $L1
      end
      get_local $l1
      i64.const 0
      i64.store offset=8
      get_local $l1
      i64.const 0
      i64.store
      i32.const 0
      set_local $l3
      loop $L4
        get_local $l1
        get_local $l3
        i32.const 2
        i32.shl
        i32.add
        tee_local $l6
        i32.load
        set_local $l2
        i32.const 1
        set_local $l0
        loop $L5
          get_local $l4
          get_local $l0
          i32.const 6
          i32.shl
          i32.add
          i32.load16_s
          get_local $l0
          i32.const 7
          i32.shl
          get_local $l3
          i32.add
          i32.const 3744
          i32.add
          i32.load8_s
          i32.mul
          get_local $l2
          i32.add
          set_local $l2
          get_local $l0
          i32.const 6
          i32.lt_u
          set_local $l8
          get_local $l0
          i32.const 2
          i32.add
          set_local $l0
          get_local $l8
          br_if $L5
        end
        get_local $l6
        get_local $l2
        i32.store
        get_local $l3
        i32.const 1
        i32.add
        tee_local $l3
        i32.const 4
        i32.ne
        br_if $L4
      end
      get_local $l1
      get_local $l4
      i32.load16_s offset=256
      i32.const 6
      i32.shl
      tee_local $l0
      get_local $l4
      i32.load16_s
      i32.const 6
      i32.shl
      tee_local $l2
      i32.add
      tee_local $l3
      get_local $l4
      i32.load16_s offset=384
      tee_local $l6
      i32.const 36
      i32.mul
      get_local $l4
      i32.load16_s offset=128
      tee_local $l8
      i32.const 83
      i32.mul
      i32.add
      tee_local $l10
      i32.sub
      i32.store offset=28
      get_local $l1
      get_local $l2
      get_local $l0
      i32.sub
      tee_local $l0
      get_local $l6
      i32.const -83
      i32.mul
      get_local $l8
      i32.const 36
      i32.mul
      i32.add
      tee_local $l2
      i32.sub
      i32.store offset=24
      get_local $l1
      get_local $l0
      get_local $l2
      i32.add
      i32.store offset=20
      get_local $l1
      get_local $l3
      get_local $l10
      i32.add
      tee_local $l0
      i32.store offset=16
      get_local $l1
      get_local $l0
      get_local $l1
      i32.load
      tee_local $l2
      i32.sub
      i32.store offset=92
      get_local $l1
      get_local $l0
      get_local $l2
      i32.add
      i32.store offset=64
      i32.const 1
      set_local $l0
      loop $L6
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $l2
        get_local $l1
        i32.const -64
        i32.sub
        i32.add
        get_local $l1
        get_local $l2
        i32.add
        i32.load
        tee_local $l3
        get_local $l1
        i32.const 16
        i32.add
        get_local $l2
        i32.add
        i32.load
        tee_local $l2
        i32.add
        i32.store
        i32.const 0
        get_local $l0
        i32.sub
        i32.const 2
        i32.shl
        get_local $l1
        i32.add
        get_local $l2
        get_local $l3
        i32.sub
        i32.store offset=92
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 4
        i32.ne
        br_if $L6
      end
      i32.const 0
      set_local $l0
      loop $L7
        get_local $l4
        get_local $l0
        i32.const 5
        i32.shl
        tee_local $l2
        i32.add
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $l3
        get_local $l1
        i32.const -64
        i32.sub
        i32.add
        i32.load
        tee_local $l6
        get_local $l1
        i32.const 32
        i32.add
        get_local $l3
        i32.add
        i32.load
        tee_local $l3
        i32.add
        i32.const -64
        i32.sub
        tee_local $l8
        i32.const 7
        i32.shr_s
        tee_local $l10
        get_local $l8
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l10
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $l4
        get_local $l2
        i32.sub
        get_local $l6
        get_local $l3
        i32.sub
        i32.const -64
        i32.sub
        tee_local $l2
        i32.const 7
        i32.shr_s
        tee_local $l3
        get_local $l2
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l3
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16 offset=480
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 8
        i32.ne
        br_if $L7
      end
      get_local $l5
      get_local $l5
      i32.const 4
      i32.sub
      get_local $l7
      i32.const 3
      i32.and
      select
      get_local $l5
      get_local $l7
      select
      get_local $l5
      get_local $l5
      i32.const 15
      i32.le_s
      select
      set_local $l5
      get_local $l4
      i32.const 2
      i32.add
      set_local $l4
      get_local $l7
      i32.const 1
      i32.add
      tee_local $l7
      i32.const 16
      i32.ne
      br_if $L0
    end
    i32.const 0
    set_local $l5
    get_local $p1
    i32.const 16
    get_local $p1
    i32.const 16
    i32.lt_s
    select
    tee_local $l7
    i32.const 2
    i32.lt_s
    set_local $l9
    loop $L8
      get_local $l1
      i64.const 0
      i64.store offset=56
      get_local $l1
      i64.const 0
      i64.store offset=48
      get_local $l1
      i64.const 0
      i64.store offset=40
      get_local $l1
      i64.const 0
      i64.store offset=32
      i32.const 0
      set_local $l3
      loop $L9
        get_local $l9
        i32.eqz
        if $I10
          get_local $l1
          i32.const 32
          i32.add
          get_local $l3
          i32.const 2
          i32.shl
          i32.add
          tee_local $p1
          i32.load
          set_local $l2
          i32.const 1
          set_local $l0
          loop $L11
            get_local $p0
            get_local $l0
            i32.const 1
            i32.shl
            i32.add
            i32.load16_s
            get_local $l0
            i32.const 6
            i32.shl
            get_local $l3
            i32.add
            i32.const 3744
            i32.add
            i32.load8_s
            i32.mul
            get_local $l2
            i32.add
            set_local $l2
            get_local $l0
            i32.const 2
            i32.add
            tee_local $l0
            get_local $l7
            i32.lt_s
            br_if $L11
          end
          get_local $p1
          get_local $l2
          i32.store
        end
        get_local $l3
        i32.const 1
        i32.add
        tee_local $l3
        i32.const 8
        i32.ne
        br_if $L9
      end
      get_local $l1
      i64.const 0
      i64.store offset=8
      get_local $l1
      i64.const 0
      i64.store
      i32.const 0
      set_local $l4
      loop $L12
        get_local $l1
        get_local $l4
        i32.const 2
        i32.shl
        i32.add
        tee_local $p1
        i32.load
        set_local $l2
        i32.const 1
        set_local $l0
        loop $L13
          get_local $p0
          get_local $l0
          i32.const 2
          i32.shl
          i32.add
          i32.load16_s
          get_local $l0
          i32.const 7
          i32.shl
          get_local $l4
          i32.add
          i32.const 3744
          i32.add
          i32.load8_s
          i32.mul
          get_local $l2
          i32.add
          set_local $l2
          get_local $l0
          i32.const 6
          i32.lt_u
          set_local $l3
          get_local $l0
          i32.const 2
          i32.add
          set_local $l0
          get_local $l3
          br_if $L13
        end
        get_local $p1
        get_local $l2
        i32.store
        get_local $l4
        i32.const 1
        i32.add
        tee_local $l4
        i32.const 4
        i32.ne
        br_if $L12
      end
      get_local $l1
      get_local $p0
      i32.load16_s offset=16
      i32.const 6
      i32.shl
      tee_local $p1
      get_local $p0
      i32.load16_s
      i32.const 6
      i32.shl
      tee_local $l4
      i32.add
      tee_local $l0
      get_local $p0
      i32.load16_s offset=24
      tee_local $l2
      i32.const 36
      i32.mul
      get_local $p0
      i32.load16_s offset=8
      tee_local $l3
      i32.const 83
      i32.mul
      i32.add
      tee_local $l6
      i32.sub
      i32.store offset=28
      get_local $l1
      get_local $l4
      get_local $p1
      i32.sub
      tee_local $p1
      get_local $l2
      i32.const -83
      i32.mul
      get_local $l3
      i32.const 36
      i32.mul
      i32.add
      tee_local $l4
      i32.sub
      i32.store offset=24
      get_local $l1
      get_local $p1
      get_local $l4
      i32.add
      i32.store offset=20
      get_local $l1
      get_local $l0
      get_local $l6
      i32.add
      tee_local $p1
      i32.store offset=16
      get_local $l1
      get_local $p1
      get_local $l1
      i32.load
      tee_local $l4
      i32.sub
      i32.store offset=92
      get_local $l1
      get_local $p1
      get_local $l4
      i32.add
      i32.store offset=64
      i32.const 1
      set_local $l0
      loop $L14
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $p1
        get_local $l1
        i32.const -64
        i32.sub
        i32.add
        get_local $p1
        get_local $l1
        i32.add
        i32.load
        tee_local $l4
        get_local $l1
        i32.const 16
        i32.add
        get_local $p1
        i32.add
        i32.load
        tee_local $p1
        i32.add
        i32.store
        i32.const 0
        get_local $l0
        i32.sub
        i32.const 2
        i32.shl
        get_local $l1
        i32.add
        get_local $p1
        get_local $l4
        i32.sub
        i32.store offset=92
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 4
        i32.ne
        br_if $L14
      end
      i32.const 0
      set_local $l0
      loop $L15
        get_local $p0
        get_local $l0
        i32.const 1
        i32.shl
        i32.add
        get_local $l0
        i32.const 2
        i32.shl
        tee_local $p1
        get_local $l1
        i32.const -64
        i32.sub
        i32.add
        i32.load
        tee_local $l4
        get_local $l1
        i32.const 32
        i32.add
        get_local $p1
        i32.add
        i32.load
        tee_local $p1
        i32.add
        i32.const 2048
        i32.add
        tee_local $l2
        i32.const 12
        i32.shr_s
        tee_local $l3
        get_local $l2
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l3
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $p0
        i32.const 15
        get_local $l0
        i32.sub
        i32.const 1
        i32.shl
        i32.add
        get_local $l4
        get_local $p1
        i32.sub
        i32.const 2048
        i32.add
        tee_local $p1
        i32.const 12
        i32.shr_s
        tee_local $l4
        get_local $p1
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l4
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 8
        i32.ne
        br_if $L15
      end
      get_local $p0
      i32.const 32
      i32.add
      set_local $p0
      get_local $l5
      i32.const 1
      i32.add
      tee_local $l5
      i32.const 16
      i32.ne
      br_if $L8
    end)
  (func $idct_8x8_8 (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32)
    get_global $g0
    i32.const 32
    i32.sub
    set_local $l3
    get_local $p1
    i32.const 4
    get_local $p1
    i32.const 4
    i32.lt_s
    select
    i32.const 4
    i32.add
    set_local $l5
    get_local $p0
    set_local $l4
    loop $L0
      get_local $l3
      i64.const 0
      i64.store offset=8
      get_local $l3
      i64.const 0
      i64.store
      i32.const 0
      set_local $l2
      loop $L1
        get_local $l5
        i32.const 2
        i32.ge_s
        if $I2
          get_local $l3
          get_local $l2
          i32.const 2
          i32.shl
          i32.add
          tee_local $l7
          i32.load
          set_local $l1
          i32.const 1
          set_local $l0
          loop $L3
            get_local $l4
            get_local $l0
            i32.const 4
            i32.shl
            i32.add
            i32.load16_s
            get_local $l0
            i32.const 7
            i32.shl
            get_local $l2
            i32.add
            i32.const 3744
            i32.add
            i32.load8_s
            i32.mul
            get_local $l1
            i32.add
            set_local $l1
            get_local $l0
            i32.const 2
            i32.add
            tee_local $l0
            get_local $l5
            i32.lt_s
            br_if $L3
          end
          get_local $l7
          get_local $l1
          i32.store
        end
        get_local $l2
        i32.const 1
        i32.add
        tee_local $l2
        i32.const 4
        i32.ne
        br_if $L1
      end
      get_local $l3
      get_local $l4
      i32.load16_s offset=64
      i32.const 6
      i32.shl
      tee_local $l0
      get_local $l4
      i32.load16_s
      i32.const 6
      i32.shl
      tee_local $l1
      i32.add
      tee_local $l2
      get_local $l4
      i32.load16_s offset=96
      tee_local $l7
      i32.const 36
      i32.mul
      get_local $l4
      i32.load16_s offset=32
      tee_local $l8
      i32.const 83
      i32.mul
      i32.add
      tee_local $l9
      i32.sub
      i32.store offset=28
      get_local $l3
      get_local $l1
      get_local $l0
      i32.sub
      tee_local $l0
      get_local $l7
      i32.const -83
      i32.mul
      get_local $l8
      i32.const 36
      i32.mul
      i32.add
      tee_local $l1
      i32.sub
      i32.store offset=24
      get_local $l3
      get_local $l0
      get_local $l1
      i32.add
      i32.store offset=20
      get_local $l3
      get_local $l2
      get_local $l9
      i32.add
      tee_local $l1
      i32.store offset=16
      i32.const 0
      set_local $l0
      loop $L4
        get_local $l4
        get_local $l0
        i32.const 4
        i32.shl
        tee_local $l2
        i32.add
        get_local $l1
        get_local $l3
        get_local $l0
        i32.const 2
        i32.shl
        i32.add
        i32.load
        tee_local $l7
        i32.add
        i32.const -64
        i32.sub
        tee_local $l8
        i32.const 7
        i32.shr_s
        tee_local $l9
        get_local $l8
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l9
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $l4
        get_local $l2
        i32.sub
        get_local $l1
        get_local $l7
        i32.sub
        i32.const -64
        i32.sub
        tee_local $l1
        i32.const 7
        i32.shr_s
        tee_local $l2
        get_local $l1
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l2
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16 offset=112
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 4
        i32.eq
        i32.eqz
        if $I5
          get_local $l3
          i32.const 16
          i32.add
          get_local $l0
          i32.const 2
          i32.shl
          i32.add
          i32.load
          set_local $l1
          br $L4
        end
      end
      get_local $l5
      get_local $l5
      i32.const 4
      i32.sub
      get_local $l6
      i32.const 3
      i32.and
      select
      get_local $l5
      get_local $l6
      select
      get_local $l5
      get_local $l5
      i32.const 7
      i32.le_s
      select
      set_local $l5
      get_local $l4
      i32.const 2
      i32.add
      set_local $l4
      get_local $l6
      i32.const 1
      i32.add
      tee_local $l6
      i32.const 8
      i32.ne
      br_if $L0
    end
    i32.const 0
    set_local $l4
    get_local $p1
    i32.const 8
    get_local $p1
    i32.const 8
    i32.lt_s
    select
    tee_local $p1
    i32.const 2
    i32.lt_s
    set_local $l5
    loop $L6
      get_local $l3
      i64.const 0
      i64.store offset=8
      get_local $l3
      i64.const 0
      i64.store
      i32.const 0
      set_local $l2
      loop $L7
        get_local $l5
        i32.eqz
        if $I8
          get_local $l3
          get_local $l2
          i32.const 2
          i32.shl
          i32.add
          tee_local $l6
          i32.load
          set_local $l1
          i32.const 1
          set_local $l0
          loop $L9
            get_local $p0
            get_local $l0
            i32.const 1
            i32.shl
            i32.add
            i32.load16_s
            get_local $l0
            i32.const 7
            i32.shl
            get_local $l2
            i32.add
            i32.const 3744
            i32.add
            i32.load8_s
            i32.mul
            get_local $l1
            i32.add
            set_local $l1
            get_local $l0
            i32.const 2
            i32.add
            tee_local $l0
            get_local $p1
            i32.lt_s
            br_if $L9
          end
          get_local $l6
          get_local $l1
          i32.store
        end
        get_local $l2
        i32.const 1
        i32.add
        tee_local $l2
        i32.const 4
        i32.ne
        br_if $L7
      end
      get_local $l3
      get_local $p0
      i32.load16_s offset=8
      i32.const 6
      i32.shl
      tee_local $l0
      get_local $p0
      i32.load16_s
      i32.const 6
      i32.shl
      tee_local $l1
      i32.add
      tee_local $l2
      get_local $p0
      i32.load16_s offset=12
      tee_local $l6
      i32.const 36
      i32.mul
      get_local $p0
      i32.load16_s offset=4
      tee_local $l7
      i32.const 83
      i32.mul
      i32.add
      tee_local $l8
      i32.sub
      i32.store offset=28
      get_local $l3
      get_local $l1
      get_local $l0
      i32.sub
      tee_local $l0
      get_local $l6
      i32.const -83
      i32.mul
      get_local $l7
      i32.const 36
      i32.mul
      i32.add
      tee_local $l1
      i32.sub
      i32.store offset=24
      get_local $l3
      get_local $l0
      get_local $l1
      i32.add
      i32.store offset=20
      get_local $l3
      get_local $l2
      get_local $l8
      i32.add
      tee_local $l1
      i32.store offset=16
      i32.const 0
      set_local $l0
      loop $L10
        get_local $p0
        get_local $l0
        i32.const 1
        i32.shl
        i32.add
        get_local $l1
        get_local $l3
        get_local $l0
        i32.const 2
        i32.shl
        i32.add
        i32.load
        tee_local $l2
        i32.add
        i32.const 2048
        i32.add
        tee_local $l6
        i32.const 12
        i32.shr_s
        tee_local $l7
        get_local $l6
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l7
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $p0
        i32.const 7
        get_local $l0
        i32.sub
        i32.const 1
        i32.shl
        i32.add
        get_local $l1
        get_local $l2
        i32.sub
        i32.const 2048
        i32.add
        tee_local $l1
        i32.const 12
        i32.shr_s
        tee_local $l2
        get_local $l1
        i32.const 31
        i32.shr_s
        i32.const 32767
        i32.xor
        get_local $l2
        i32.const 32768
        i32.add
        i32.const 65536
        i32.lt_u
        select
        i32.store16
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 4
        i32.eq
        i32.eqz
        if $I11
          get_local $l3
          i32.const 16
          i32.add
          get_local $l0
          i32.const 2
          i32.shl
          i32.add
          i32.load
          set_local $l1
          br $L10
        end
      end
      get_local $p0
      i32.const 16
      i32.add
      set_local $p0
      get_local $l4
      i32.const 1
      i32.add
      tee_local $l4
      i32.const 8
      i32.ne
      br_if $L6
    end)
  (func $transform_add32x32_8 (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    loop $L0
      i32.const 0
      set_local $l1
      loop $L1
        get_local $p0
        get_local $l1
        i32.add
        tee_local $l0
        get_local $p1
        i32.load16_s
        get_local $l0
        i32.load8_u
        i32.add
        tee_local $l0
        i32.const -1
        i32.const 0
        get_local $l0
        i32.const 0
        i32.gt_s
        select
        get_local $l0
        i32.const 256
        i32.lt_u
        select
        i32.store8
        get_local $p1
        i32.const 2
        i32.add
        set_local $p1
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 32
        i32.ne
        br_if $L1
      end
      get_local $p0
      get_local $p2
      i32.add
      set_local $p0
      get_local $l2
      i32.const 1
      i32.add
      tee_local $l2
      i32.const 32
      i32.ne
      br_if $L0
    end)
  (func $transform_add16x16_8 (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    loop $L0
      i32.const 0
      set_local $l1
      loop $L1
        get_local $p0
        get_local $l1
        i32.add
        tee_local $l0
        get_local $p1
        i32.load16_s
        get_local $l0
        i32.load8_u
        i32.add
        tee_local $l0
        i32.const -1
        i32.const 0
        get_local $l0
        i32.const 0
        i32.gt_s
        select
        get_local $l0
        i32.const 256
        i32.lt_u
        select
        i32.store8
        get_local $p1
        i32.const 2
        i32.add
        set_local $p1
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 16
        i32.ne
        br_if $L1
      end
      get_local $p0
      get_local $p2
      i32.add
      set_local $p0
      get_local $l2
      i32.const 1
      i32.add
      tee_local $l2
      i32.const 16
      i32.ne
      br_if $L0
    end)
  (func $transform_add8x8_8 (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    loop $L0
      i32.const 0
      set_local $l1
      loop $L1
        get_local $p0
        get_local $l1
        i32.add
        tee_local $l0
        get_local $p1
        i32.load16_s
        get_local $l0
        i32.load8_u
        i32.add
        tee_local $l0
        i32.const -1
        i32.const 0
        get_local $l0
        i32.const 0
        i32.gt_s
        select
        get_local $l0
        i32.const 256
        i32.lt_u
        select
        i32.store8
        get_local $p1
        i32.const 2
        i32.add
        set_local $p1
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 8
        i32.ne
        br_if $L1
      end
      get_local $p0
      get_local $p2
      i32.add
      set_local $p0
      get_local $l2
      i32.const 1
      i32.add
      tee_local $l2
      i32.const 8
      i32.ne
      br_if $L0
    end)
  (func $sao_edge_filter_8 (type $t14) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (param $p8 i32) (param $p9 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32)
    get_local $p6
    get_local $p9
    i32.gt_s
    if $I0
      get_local $p4
      get_local $p7
      i32.const 2
      i32.shl
      i32.add
      i32.load offset=100
      i32.const 2
      i32.shl
      tee_local $l0
      i32.const 2658
      i32.add
      i32.load8_s
      set_local $l5
      get_local $l0
      i32.const 2656
      i32.add
      i32.load8_s
      set_local $l6
      get_local $p3
      get_local $p9
      i32.mul
      set_local $l1
      get_local $p2
      get_local $p9
      i32.mul
      set_local $l2
      get_local $l0
      i32.const 2657
      i32.add
      i32.load8_s
      get_local $p9
      i32.add
      get_local $p3
      i32.mul
      set_local $l3
      get_local $l0
      i32.const 2659
      i32.add
      i32.load8_s
      get_local $p9
      i32.add
      get_local $p3
      i32.mul
      set_local $l0
      get_local $p4
      get_local $p7
      i32.const 10
      i32.mul
      i32.add
      set_local $l7
      loop $L1
        get_local $p5
        get_local $p8
        i32.gt_s
        if $I2
          get_local $l0
          get_local $l5
          i32.add
          set_local $l8
          get_local $l3
          get_local $l6
          i32.add
          set_local $l9
          get_local $p8
          set_local $p4
          loop $L3
            get_local $p0
            get_local $p4
            get_local $l2
            i32.add
            i32.add
            get_local $l7
            i32.const 1
            i32.const -1
            i32.const 0
            get_local $p1
            get_local $p4
            get_local $l1
            i32.add
            i32.add
            i32.load8_u
            tee_local $p7
            get_local $p1
            get_local $p4
            get_local $l8
            i32.add
            i32.add
            i32.load8_u
            tee_local $l4
            i32.ne
            select
            get_local $p7
            get_local $l4
            i32.gt_u
            select
            i32.const 3
            i32.const 2
            i32.const 1
            get_local $p7
            get_local $p1
            get_local $p4
            get_local $l9
            i32.add
            i32.add
            i32.load8_u
            tee_local $l4
            i32.eq
            select
            get_local $p7
            get_local $l4
            i32.gt_u
            select
            i32.add
            i32.const 2640
            i32.add
            i32.load8_u
            i32.const 1
            i32.shl
            i32.add
            i32.load16_s offset=112
            get_local $p7
            i32.add
            tee_local $p7
            i32.const -1
            i32.const 0
            get_local $p7
            i32.const 0
            i32.gt_s
            select
            get_local $p7
            i32.const 256
            i32.lt_u
            select
            i32.store8
            get_local $p4
            i32.const 1
            i32.add
            tee_local $p4
            get_local $p5
            i32.ne
            br_if $L3
          end
        end
        get_local $p3
        get_local $l0
        i32.add
        set_local $l0
        get_local $p3
        get_local $l3
        i32.add
        set_local $l3
        get_local $p2
        get_local $l2
        i32.add
        set_local $l2
        get_local $p3
        get_local $l1
        i32.add
        set_local $l1
        get_local $p9
        i32.const 1
        i32.add
        tee_local $p9
        get_local $p6
        i32.ne
        br_if $L1
      end
    end)
  (func $hevc_loop_filter_luma_8 (type $t10) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32) (local $l13 i32) (local $l14 i32) (local $l15 i32) (local $l16 i32) (local $l17 i32) (local $l18 i32) (local $l19 i32) (local $l20 i32) (local $l21 i32) (local $l22 i32) (local $l23 i32) (local $l24 i32) (local $l25 i32) (local $l26 i32) (local $l27 i32) (local $l28 i32) (local $l29 i32) (local $l30 i32) (local $l31 i32) (local $l32 i32) (local $l33 i32) (local $l34 i32) (local $l35 i32) (local $l36 i32) (local $l37 i32) (local $l38 i32)
    get_local $p2
    i32.const 3
    i32.mul
    tee_local $l13
    get_local $p1
    i32.const 3
    i32.mul
    tee_local $l25
    i32.add
    set_local $l31
    get_local $l13
    get_local $p1
    i32.const 2
    i32.shl
    tee_local $l0
    i32.sub
    set_local $l32
    get_local $l13
    get_local $p1
    i32.const 1
    i32.shl
    tee_local $l15
    i32.add
    set_local $l33
    get_local $l13
    get_local $l15
    i32.sub
    set_local $l34
    get_local $l13
    get_local $p1
    i32.const -3
    i32.mul
    tee_local $l21
    i32.add
    set_local $l35
    get_local $p3
    i32.const 2
    i32.shr_s
    set_local $l26
    get_local $p3
    i32.const 3
    i32.shr_s
    set_local $l27
    get_local $p2
    i32.const 2
    i32.shl
    set_local $l36
    i32.const 0
    get_local $p1
    i32.sub
    set_local $l19
    i32.const 0
    get_local $l0
    i32.sub
    set_local $l28
    get_local $p1
    get_local $l13
    i32.add
    set_local $l37
    get_local $l13
    get_local $p1
    i32.sub
    set_local $l38
    i32.const 0
    get_local $l15
    i32.sub
    set_local $l20
    get_local $p3
    i32.const 1
    i32.shr_s
    get_local $p3
    i32.add
    i32.const 3
    i32.shr_s
    set_local $l29
    i32.const 1
    set_local $l30
    loop $L0
      block $B1
        get_local $p3
        get_local $p0
        get_local $l13
        i32.add
        i32.load8_u
        tee_local $l7
        get_local $p0
        get_local $l33
        i32.add
        i32.load8_u
        get_local $p0
        get_local $l37
        i32.add
        i32.load8_u
        i32.const 1
        i32.shl
        i32.sub
        i32.add
        tee_local $l0
        get_local $l0
        i32.const 31
        i32.shr_s
        tee_local $l0
        i32.add
        get_local $l0
        i32.xor
        tee_local $l3
        get_local $p0
        get_local $l38
        i32.add
        i32.load8_u
        tee_local $l8
        get_local $p0
        get_local $l35
        i32.add
        i32.load8_u
        get_local $p0
        get_local $l34
        i32.add
        i32.load8_u
        i32.const 1
        i32.shl
        i32.sub
        i32.add
        tee_local $l0
        get_local $l0
        i32.const 31
        i32.shr_s
        tee_local $l0
        i32.add
        get_local $l0
        i32.xor
        tee_local $l4
        i32.add
        tee_local $l16
        get_local $p0
        i32.load8_u
        tee_local $l0
        get_local $p0
        get_local $l15
        i32.add
        i32.load8_u
        tee_local $l11
        get_local $p0
        get_local $p1
        i32.add
        i32.load8_u
        tee_local $l14
        i32.const 1
        i32.shl
        i32.sub
        i32.add
        tee_local $l1
        get_local $l1
        i32.const 31
        i32.shr_s
        tee_local $l1
        i32.add
        get_local $l1
        i32.xor
        tee_local $l17
        get_local $p0
        get_local $l19
        i32.add
        i32.load8_u
        tee_local $l1
        get_local $p0
        get_local $l21
        i32.add
        i32.load8_u
        tee_local $l12
        get_local $p0
        get_local $l20
        i32.add
        i32.load8_u
        tee_local $l10
        i32.const 1
        i32.shl
        i32.sub
        i32.add
        tee_local $l5
        get_local $l5
        i32.const 31
        i32.shr_s
        tee_local $l5
        i32.add
        get_local $l5
        i32.xor
        tee_local $l18
        i32.add
        tee_local $l22
        i32.add
        i32.le_s
        if $I2
          get_local $p0
          get_local $l36
          i32.add
          set_local $p0
          br $B1
        end
        get_local $p6
        get_local $l2
        i32.add
        i32.load8_u
        set_local $l23
        get_local $p5
        get_local $l2
        i32.add
        i32.load8_u
        set_local $l24
        get_local $p4
        get_local $l2
        i32.const 2
        i32.shl
        i32.add
        i32.load
        set_local $l5
        block $B3
          get_local $p0
          get_local $l25
          i32.add
          i32.load8_u
          tee_local $l6
          get_local $l0
          i32.sub
          tee_local $l2
          get_local $l2
          i32.const 31
          i32.shr_s
          tee_local $l2
          i32.add
          get_local $l2
          i32.xor
          get_local $p0
          get_local $l28
          i32.add
          i32.load8_u
          get_local $l1
          i32.sub
          tee_local $l2
          get_local $l2
          i32.const 31
          i32.shr_s
          tee_local $l2
          i32.add
          get_local $l2
          i32.xor
          i32.add
          get_local $l27
          i32.ge_s
          br_if $B3
          get_local $l5
          i32.const 5
          i32.mul
          i32.const 1
          i32.add
          i32.const 1
          i32.shr_s
          tee_local $l2
          get_local $l1
          get_local $l0
          i32.sub
          tee_local $l9
          get_local $l9
          i32.const 31
          i32.shr_s
          tee_local $l9
          i32.add
          get_local $l9
          i32.xor
          i32.le_s
          br_if $B3
          get_local $p0
          get_local $l31
          i32.add
          i32.load8_u
          get_local $l7
          i32.sub
          tee_local $l9
          get_local $l9
          i32.const 31
          i32.shr_s
          tee_local $l9
          i32.add
          get_local $l9
          i32.xor
          get_local $p0
          get_local $l32
          i32.add
          i32.load8_u
          get_local $l8
          i32.sub
          tee_local $l9
          get_local $l9
          i32.const 31
          i32.shr_s
          tee_local $l9
          i32.add
          get_local $l9
          i32.xor
          i32.add
          get_local $l27
          i32.ge_s
          get_local $l22
          i32.const 1
          i32.shl
          get_local $l26
          i32.ge_s
          i32.or
          br_if $B3
          get_local $l2
          get_local $l8
          get_local $l7
          i32.sub
          tee_local $l2
          get_local $l2
          i32.const 31
          i32.shr_s
          tee_local $l2
          i32.add
          get_local $l2
          i32.xor
          i32.le_s
          get_local $l16
          i32.const 1
          i32.shl
          get_local $l26
          i32.ge_s
          i32.or
          br_if $B3
          i32.const 0
          set_local $l16
          i32.const 0
          get_local $l5
          i32.const 1
          i32.shl
          tee_local $l3
          i32.sub
          set_local $l4
          loop $L4
            get_local $l14
            set_local $l5
            get_local $l0
            set_local $l7
            get_local $l1
            set_local $l8
            get_local $l10
            set_local $l2
            get_local $l24
            i32.eqz
            if $I5
              get_local $p0
              get_local $l28
              i32.add
              i32.load8_u
              set_local $l22
              get_local $p0
              get_local $l19
              i32.add
              get_local $l1
              get_local $l4
              get_local $l3
              get_local $l5
              get_local $l12
              i32.add
              get_local $l7
              get_local $l8
              i32.add
              tee_local $l17
              get_local $l2
              i32.add
              i32.const 1
              i32.shl
              i32.add
              i32.const 4
              i32.add
              i32.const 3
              i32.shr_u
              get_local $l8
              i32.sub
              tee_local $l18
              get_local $l3
              get_local $l18
              i32.lt_s
              select
              get_local $l4
              get_local $l18
              i32.gt_s
              select
              i32.add
              i32.store8
              get_local $p0
              get_local $l20
              i32.add
              get_local $l10
              get_local $l4
              get_local $l3
              get_local $l12
              get_local $l17
              i32.add
              get_local $l2
              i32.add
              i32.const 2
              i32.add
              i32.const 2
              i32.shr_u
              get_local $l2
              i32.sub
              tee_local $l1
              get_local $l1
              get_local $l3
              i32.gt_s
              select
              get_local $l1
              get_local $l4
              i32.lt_s
              select
              i32.add
              i32.store8
              get_local $p0
              get_local $l21
              i32.add
              get_local $l12
              get_local $l4
              get_local $l3
              get_local $l17
              get_local $l12
              i32.const 3
              i32.mul
              i32.add
              get_local $l2
              i32.add
              get_local $l22
              i32.const 1
              i32.shl
              i32.add
              i32.const 4
              i32.add
              i32.const 3
              i32.shr_u
              get_local $l12
              i32.sub
              tee_local $l1
              get_local $l1
              get_local $l3
              i32.gt_s
              select
              get_local $l1
              get_local $l4
              i32.lt_s
              select
              i32.add
              i32.store8
            end
            get_local $l23
            i32.eqz
            if $I6
              get_local $p0
              get_local $l0
              get_local $l4
              get_local $l3
              get_local $l2
              get_local $l11
              i32.add
              get_local $l7
              get_local $l8
              i32.add
              get_local $l5
              i32.add
              tee_local $l1
              i32.const 1
              i32.shl
              i32.add
              i32.const 4
              i32.add
              i32.const 3
              i32.shr_u
              get_local $l7
              i32.sub
              tee_local $l10
              get_local $l3
              get_local $l10
              i32.lt_s
              select
              get_local $l4
              get_local $l10
              i32.gt_s
              select
              i32.add
              i32.store8
              get_local $p0
              get_local $p1
              i32.add
              get_local $l14
              get_local $l4
              get_local $l3
              get_local $l1
              get_local $l11
              i32.add
              i32.const 2
              i32.add
              i32.const 2
              i32.shr_u
              get_local $l5
              i32.sub
              tee_local $l0
              get_local $l0
              get_local $l3
              i32.gt_s
              select
              get_local $l0
              get_local $l4
              i32.lt_s
              select
              i32.add
              i32.store8
              get_local $p0
              get_local $l15
              i32.add
              get_local $l11
              get_local $l4
              get_local $l3
              get_local $l1
              get_local $l11
              i32.const 3
              i32.mul
              i32.add
              get_local $l6
              i32.const 1
              i32.shl
              i32.add
              i32.const 4
              i32.add
              i32.const 3
              i32.shr_u
              get_local $l11
              i32.sub
              tee_local $l0
              get_local $l0
              get_local $l3
              i32.gt_s
              select
              get_local $l0
              get_local $l4
              i32.lt_s
              select
              i32.add
              i32.store8
            end
            get_local $p0
            get_local $p2
            i32.add
            set_local $p0
            get_local $l16
            i32.const 1
            i32.add
            tee_local $l16
            i32.const 4
            i32.eq
            br_if $B1
            get_local $p0
            get_local $l25
            i32.add
            i32.load8_u
            set_local $l6
            get_local $p0
            get_local $l15
            i32.add
            i32.load8_u
            set_local $l11
            get_local $p0
            get_local $p1
            i32.add
            i32.load8_u
            set_local $l14
            get_local $p0
            get_local $l19
            i32.add
            i32.load8_u
            set_local $l1
            get_local $p0
            get_local $l20
            i32.add
            i32.load8_u
            set_local $l10
            get_local $p0
            get_local $l21
            i32.add
            i32.load8_u
            set_local $l12
            get_local $p0
            i32.load8_u
            set_local $l0
            br $L4
          end
          unreachable
        end
        get_local $l23
        i32.eqz
        get_local $l3
        get_local $l17
        i32.add
        get_local $l29
        i32.lt_s
        i32.and
        set_local $l16
        get_local $l24
        i32.eqz
        get_local $l4
        get_local $l18
        i32.add
        get_local $l29
        i32.lt_s
        i32.and
        set_local $l17
        i32.const 0
        set_local $l2
        i32.const 0
        get_local $l5
        i32.sub
        set_local $l4
        get_local $l5
        i32.const 10
        i32.mul
        set_local $l18
        i32.const 0
        get_local $l5
        i32.const 1
        i32.shr_s
        tee_local $l7
        i32.sub
        set_local $l8
        loop $L7
          block $B8
            get_local $l0
            get_local $l1
            i32.sub
            i32.const 9
            i32.mul
            get_local $l14
            get_local $l10
            i32.sub
            i32.const -3
            i32.mul
            i32.add
            i32.const 8
            i32.add
            tee_local $l6
            i32.const 4
            i32.shr_s
            tee_local $l3
            get_local $l6
            i32.const 31
            i32.shr_s
            tee_local $l6
            i32.add
            get_local $l6
            i32.xor
            get_local $l18
            i32.ge_s
            br_if $B8
            get_local $l4
            get_local $l5
            get_local $l3
            get_local $l3
            get_local $l5
            i32.gt_s
            select
            get_local $l3
            get_local $l4
            i32.lt_s
            select
            set_local $l3
            get_local $l24
            i32.eqz
            if $I9
              get_local $p0
              get_local $l19
              i32.add
              get_local $l1
              get_local $l3
              i32.add
              tee_local $l6
              i32.const -1
              i32.const 0
              get_local $l6
              i32.const 0
              i32.gt_s
              select
              get_local $l6
              i32.const 256
              i32.lt_u
              select
              i32.store8
            end
            get_local $l23
            i32.eqz
            if $I10
              get_local $p0
              get_local $l0
              get_local $l3
              i32.sub
              tee_local $l6
              i32.const -1
              i32.const 0
              get_local $l6
              i32.const 0
              i32.gt_s
              select
              get_local $l6
              i32.const 256
              i32.lt_u
              select
              i32.store8
            end
            get_local $l17
            if $I11
              get_local $p0
              get_local $l20
              i32.add
              get_local $l8
              get_local $l7
              get_local $l1
              get_local $l12
              i32.add
              i32.const 1
              i32.add
              i32.const 1
              i32.shr_u
              get_local $l10
              i32.sub
              get_local $l3
              i32.add
              i32.const 1
              i32.shr_s
              tee_local $l1
              get_local $l1
              get_local $l7
              i32.gt_s
              select
              get_local $l1
              get_local $l8
              i32.lt_s
              select
              get_local $l10
              i32.add
              tee_local $l1
              i32.const -1
              i32.const 0
              get_local $l1
              i32.const 0
              i32.gt_s
              select
              get_local $l1
              i32.const 256
              i32.lt_u
              select
              i32.store8
            end
            get_local $l16
            i32.eqz
            br_if $B8
            get_local $p0
            get_local $p1
            i32.add
            get_local $l8
            get_local $l7
            get_local $l0
            get_local $l11
            i32.add
            i32.const 1
            i32.add
            i32.const 1
            i32.shr_u
            get_local $l14
            i32.sub
            get_local $l3
            i32.sub
            i32.const 1
            i32.shr_s
            tee_local $l0
            get_local $l0
            get_local $l7
            i32.gt_s
            select
            get_local $l0
            get_local $l8
            i32.lt_s
            select
            get_local $l14
            i32.add
            tee_local $l0
            i32.const -1
            i32.const 0
            get_local $l0
            i32.const 0
            i32.gt_s
            select
            get_local $l0
            i32.const 256
            i32.lt_u
            select
            i32.store8
          end
          get_local $p0
          get_local $p2
          i32.add
          set_local $p0
          get_local $l2
          i32.const 1
          i32.add
          tee_local $l2
          i32.const 4
          i32.eq
          br_if $B1
          get_local $p0
          get_local $l15
          i32.add
          i32.load8_u
          set_local $l11
          get_local $p0
          get_local $p1
          i32.add
          i32.load8_u
          set_local $l14
          get_local $p0
          get_local $l19
          i32.add
          i32.load8_u
          set_local $l1
          get_local $p0
          get_local $l20
          i32.add
          i32.load8_u
          set_local $l10
          get_local $p0
          get_local $l21
          i32.add
          i32.load8_u
          set_local $l12
          get_local $p0
          i32.load8_u
          set_local $l0
          br $L7
        end
        unreachable
      end
      i32.const 1
      set_local $l2
      get_local $l30
      set_local $l0
      i32.const 0
      set_local $l30
      get_local $l0
      br_if $L0
    end)
  (func $hevc_loop_filter_chroma_8 (type $t6) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32)
    get_local $p2
    i32.const 2
    i32.shl
    set_local $l6
    i32.const 0
    get_local $p1
    i32.sub
    set_local $l7
    i32.const 0
    get_local $p1
    i32.const 1
    i32.shl
    i32.sub
    set_local $l8
    i32.const 1
    set_local $l3
    loop $L0
      block $B1
        get_local $p3
        get_local $l0
        i32.const 2
        i32.shl
        i32.add
        i32.load
        tee_local $l1
        i32.const 0
        i32.le_s
        if $I2
          get_local $p0
          get_local $l6
          i32.add
          set_local $p0
          br $B1
        end
        i32.const 0
        set_local $l4
        i32.const 0
        get_local $l1
        i32.sub
        set_local $l5
        get_local $p5
        get_local $l0
        i32.add
        i32.load8_u
        set_local $l9
        get_local $p4
        get_local $l0
        i32.add
        i32.load8_u
        set_local $l10
        loop $L3
          get_local $l5
          get_local $l1
          get_local $p0
          get_local $l8
          i32.add
          i32.load8_u
          get_local $p0
          get_local $p1
          i32.add
          i32.load8_u
          i32.sub
          get_local $p0
          i32.load8_u
          tee_local $l11
          get_local $p0
          get_local $l7
          i32.add
          tee_local $l2
          i32.load8_u
          tee_local $l12
          i32.sub
          i32.const 2
          i32.shl
          i32.add
          i32.const 4
          i32.add
          i32.const 3
          i32.shr_s
          tee_local $l0
          get_local $l0
          get_local $l1
          i32.gt_s
          select
          get_local $l0
          get_local $l5
          i32.lt_s
          select
          set_local $l0
          get_local $l10
          i32.eqz
          if $I4
            get_local $l2
            get_local $l0
            get_local $l12
            i32.add
            tee_local $l2
            i32.const -1
            i32.const 0
            get_local $l2
            i32.const 0
            i32.gt_s
            select
            get_local $l2
            i32.const 256
            i32.lt_u
            select
            i32.store8
          end
          get_local $l9
          i32.eqz
          if $I5
            get_local $p0
            get_local $l11
            get_local $l0
            i32.sub
            tee_local $l0
            i32.const -1
            i32.const 0
            get_local $l0
            i32.const 0
            i32.gt_s
            select
            get_local $l0
            i32.const 256
            i32.lt_u
            select
            i32.store8
          end
          get_local $p0
          get_local $p2
          i32.add
          set_local $p0
          get_local $l4
          i32.const 1
          i32.add
          tee_local $l4
          i32.const 4
          i32.ne
          br_if $L3
        end
      end
      i32.const 1
      set_local $l0
      get_local $l3
      set_local $l1
      i32.const 0
      set_local $l3
      get_local $l1
      br_if $L0
    end)
  (func $ff_hevc_set_neighbour_available (type $t8) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32)
    get_local $p0
    i32.load offset=136
    tee_local $l0
    i32.const 31296
    i32.add
    i32.const -1
    get_local $p0
    i32.load offset=200
    i32.load offset=13080
    tee_local $l3
    i32.shl
    i32.const -1
    i32.xor
    tee_local $l1
    get_local $p2
    i32.and
    tee_local $l4
    get_local $l0
    i32.load8_u offset=309
    i32.or
    i32.const 0
    i32.ne
    tee_local $p0
    i32.store
    get_local $l0
    i32.const 31292
    i32.add
    get_local $p1
    get_local $l1
    i32.and
    tee_local $l5
    get_local $l0
    i32.load8_u offset=308
    i32.or
    i32.const 0
    i32.ne
    tee_local $l2
    i32.store
    get_local $l0
    i32.const 31300
    i32.add
    block $B0 (result i32)
      get_local $p1
      get_local $p2
      i32.or
      get_local $l1
      i32.and
      i32.eqz
      if $I1
        get_local $l0
        i32.load8_u offset=311
        br $B0
      end
      get_local $p0
      i32.const 0
      get_local $l2
      select
    end
    i32.store
    get_local $p3
    get_local $l5
    i32.add
    i32.const 1
    get_local $l3
    i32.shl
    i32.eq
    if $I2
      get_local $l4
      i32.eqz
      get_local $l0
      i32.load8_u offset=310
      i32.const 0
      i32.ne
      i32.and
      set_local $p0
    end
    get_local $l0
    i32.const 31308
    i32.add
    get_local $p0
    i32.store
    i32.const 0
    set_local $l1
    get_local $l0
    i32.const 31304
    i32.add
    get_local $p0
    if $I3 (result i32)
      get_local $l0
      i32.load offset=312
      get_local $p1
      get_local $p3
      i32.add
      i32.gt_s
    else
      get_local $l1
    end
    i32.store
    get_local $l0
    get_local $l2
    i32.const 0
    get_local $l0
    i32.load offset=316
    get_local $p2
    get_local $p4
    i32.add
    i32.gt_s
    select
    i32.store offset=31288)
  (func $ff_hevc_decode_nal_sps (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l10
    set_global $g0
    get_local $p0
    i32.load offset=136
    set_local $l4
    get_local $l10
    i32.const 13196
    call $av_buffer_allocz
    tee_local $l8
    i32.store offset=12
    block $B0
      get_local $l8
      i32.eqz
      if $I1
        i32.const -48
        set_local $l3
        br $B0
      end
      get_local $l8
      i32.load offset=4
      set_local $l0
      get_local $p0
      call $create_dummy_vps
      tee_local $l3
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $l0
      i32.const 1
      i32.store offset=72
      get_local $l0
      i32.const 0
      i32.store
      get_local $l0
      get_local $l4
      i32.const 204
      i32.add
      tee_local $l2
      i32.const 8
      call $get_bits
      tee_local $l4
      i32.store offset=4
      i32.const -1094995529
      set_local $l3
      block $B2
        get_local $l4
        i32.const 3
        i32.gt_s
        br_if $B2
        get_local $l0
        i32.const 0
        i32.store8 offset=8
        get_local $l0
        get_local $l2
        i32.const 32
        call $get_bits_long
        i32.store offset=13120
        get_local $l0
        get_local $l2
        i32.const 32
        call $get_bits_long
        tee_local $l4
        i32.store offset=13124
        block $B3
          get_local $l0
          i32.load offset=13120
          get_local $l4
          get_local $p0
          i32.load offset=4
          call $av_image_check_size
          tee_local $l4
          i32.const 0
          i32.lt_s
          br_if $B3
          get_local $l0
          get_local $l2
          i32.const 8
          call $get_bits
          tee_local $l1
          i32.const 8
          i32.add
          i32.store offset=52
          get_local $l1
          br_if $B2
          i32.const 5
          set_local $l1
          get_local $l0
          i32.load offset=4
          tee_local $l5
          i32.const 2
          i32.le_u
          if $I4
            get_local $l5
            i32.const 2
            i32.shl
            i32.const 2800
            i32.add
            i32.load
            set_local $l1
          end
          i32.const 0
          set_local $l5
          get_local $l0
          i32.const 0
          i32.store offset=56
          get_local $l0
          get_local $l1
          i32.store offset=60
          get_local $l1
          call $av_pix_fmt_desc_get
          tee_local $l1
          i32.eqz
          if $I5
            i32.const -28
            set_local $l3
            br $B2
          end
          get_local $l0
          i32.const 0
          i32.store offset=13168
          get_local $l0
          i32.const 0
          i32.store offset=13180
          get_local $l0
          i32.const 13176
          i32.add
          get_local $l1
          i32.load8_u offset=5
          tee_local $l6
          i32.store
          get_local $l0
          i32.const 13172
          i32.add
          get_local $l6
          i32.store
          get_local $l0
          i32.const 13188
          i32.add
          get_local $l1
          i32.load8_u offset=6
          tee_local $l1
          i32.store
          get_local $l0
          i32.const 13184
          i32.add
          get_local $l1
          i32.store
          get_local $l0
          i32.const 8
          i32.store offset=64
          get_local $l0
          i32.load offset=72
          tee_local $l1
          i32.const 1
          i32.ge_s
          if $I6
            loop $L7
              get_local $l0
              get_local $l5
              i32.const 12
              i32.mul
              i32.add
              tee_local $l6
              i32.const -1
              i32.store offset=84
              get_local $l6
              i64.const 1
              i64.store offset=76 align=4
              get_local $l5
              i32.const 1
              i32.add
              tee_local $l5
              get_local $l1
              i32.ne
              br_if $L7
            end
          end
          get_local $l0
          get_local $l2
          call $get_ue_golomb_long
          i32.const 3
          i32.add
          tee_local $l1
          i32.store offset=13064
          get_local $l0
          i32.const -1
          get_local $l1
          i32.shl
          tee_local $l1
          i32.const -1
          i32.xor
          tee_local $l5
          get_local $l0
          i32.load offset=13120
          i32.add
          get_local $l1
          i32.and
          i32.store offset=13120
          get_local $l0
          get_local $l0
          i32.load offset=13124
          get_local $l5
          i32.add
          get_local $l1
          i32.and
          i32.store offset=13124
          get_local $l0
          get_local $l2
          call $get_ue_golomb_long
          i32.store offset=13068
          get_local $l0
          get_local $l2
          call $get_ue_golomb_long
          i32.const 2
          i32.add
          i32.store offset=13072
          get_local $l0
          get_local $l2
          call $get_ue_golomb_long
          get_local $l0
          i32.load offset=13072
          tee_local $l1
          i32.add
          i32.store offset=13076
          get_local $l1
          get_local $l0
          i32.load offset=13064
          i32.ge_u
          br_if $B2
          get_local $l0
          get_local $l2
          call $get_ue_golomb_long
          tee_local $l1
          i32.store offset=13088
          get_local $l0
          get_local $l1
          i32.store offset=13092
          get_local $l0
          i32.const 1
          i32.store8 offset=12940
          get_local $l0
          get_local $l2
          call $get_bits1
          i32.store8 offset=12941
          get_local $l0
          get_local $l2
          call $get_bits1
          tee_local $l1
          i32.store offset=68
          get_local $l1
          if $I8
            get_local $l0
            get_local $l2
            i32.const 4
            call $get_bits
            i32.const 1
            i32.add
            i32.store8 offset=13044
            get_local $l0
            i32.const 13045
            i32.add
            get_local $l2
            i32.const 4
            call $get_bits
            i32.const 1
            i32.add
            i32.store8
            get_local $l0
            i32.const 13048
            i32.add
            get_local $l2
            call $get_ue_golomb_long
            i32.const 3
            i32.add
            tee_local $l1
            i32.store
            get_local $l0
            i32.const 13052
            i32.add
            get_local $l2
            call $get_ue_golomb_long
            get_local $l1
            i32.add
            i32.store
            get_local $l0
            i32.load offset=52
            get_local $l0
            i32.load8_u offset=13044
            i32.lt_s
            br_if $B2
            get_local $l0
            i32.const 13056
            i32.add
            get_local $l2
            call $get_bits1
            i32.store8
          end
          get_local $l0
          i32.const 1
          i32.store8 offset=13060
          get_local $l0
          i32.const 0
          i32.store8 offset=12942
          get_local $l0
          i32.const 0
          i32.store offset=2184
          get_local $l2
          call $get_bits1
          set_local $l3
          get_local $l0
          i64.const 4294967296
          i64.store offset=160 align=4
          get_local $l0
          get_local $l3
          i32.store8 offset=13061
          block $B9
            get_local $l2
            call $get_bits1
            i32.eqz
            br_if $B9
            get_local $l2
            call $get_bits1
            set_local $l3
            get_local $l2
            i32.const 7
            call $skip_bits
            get_local $l3
            i32.eqz
            br_if $B9
            get_local $l0
            get_local $l2
            call $get_bits1
            i32.store offset=13096
            get_local $l0
            get_local $l2
            call $get_bits1
            i32.store offset=13100
            get_local $l0
            get_local $l2
            call $get_bits1
            i32.store offset=13104
            get_local $l0
            get_local $l2
            call $get_bits1
            i32.store offset=13108
            get_local $l2
            call $get_bits1
            drop
            get_local $l0
            get_local $l2
            call $get_bits1
            i32.store offset=13112
            get_local $l2
            call $get_bits1
            drop
            get_local $l0
            get_local $l2
            call $get_bits1
            i32.store offset=13116
            get_local $l2
            call $get_bits1
            drop
          end
          get_local $l0
          get_local $l0
          i32.load offset=13120
          tee_local $l1
          i32.store offset=12
          get_local $l0
          get_local $l0
          i32.load offset=13124
          tee_local $l5
          i32.store offset=16
          get_local $l0
          get_local $l5
          get_local $l0
          i32.load offset=13064
          tee_local $l6
          i32.shr_s
          i32.store offset=13144
          get_local $l0
          get_local $l1
          get_local $l6
          i32.shr_s
          i32.store offset=13140
          get_local $l0
          get_local $l6
          i32.const 1
          i32.sub
          tee_local $l9
          i32.store offset=13084
          get_local $l0
          get_local $l6
          get_local $l0
          i32.load offset=13068
          i32.add
          tee_local $l3
          i32.store offset=13080
          get_local $l0
          get_local $l5
          get_local $l0
          i32.load offset=13072
          tee_local $l7
          i32.shr_s
          i32.store offset=13152
          get_local $l0
          get_local $l1
          get_local $l7
          i32.shr_s
          i32.store offset=13148
          get_local $l0
          get_local $l5
          get_local $l9
          i32.shr_s
          i32.store offset=13160
          get_local $l0
          get_local $l1
          get_local $l9
          i32.shr_s
          i32.store offset=13156
          get_local $l0
          i32.const -1
          get_local $l3
          get_local $l7
          i32.sub
          tee_local $l9
          i32.shl
          i32.const -1
          i32.xor
          i32.store offset=13164
          get_local $l0
          get_local $l5
          i32.const -1
          get_local $l3
          i32.shl
          i32.const -1
          i32.xor
          tee_local $l7
          i32.add
          get_local $l3
          i32.shr_s
          tee_local $l11
          i32.store offset=13132
          get_local $l0
          get_local $l1
          get_local $l7
          i32.add
          get_local $l3
          i32.shr_s
          tee_local $l7
          i32.store offset=13128
          get_local $l0
          get_local $l7
          get_local $l11
          i32.mul
          i32.store offset=13136
          get_local $l0
          get_local $l0
          i32.load offset=52
          i32.const 6
          i32.mul
          i32.const 48
          i32.sub
          i32.store offset=13192
          get_local $l1
          get_local $l5
          i32.or
          i32.const -1
          get_local $l6
          i32.shl
          i32.const -1
          i32.xor
          i32.and
          get_local $l3
          i32.const 6
          i32.gt_u
          i32.or
          br_if $B3
          get_local $l0
          i32.load offset=13088
          get_local $l9
          i32.gt_u
          br_if $B3
          get_local $l0
          i32.load offset=13092
          get_local $l9
          i32.gt_u
          br_if $B3
          get_local $l0
          i32.load offset=13076
          get_local $l3
          i32.const 5
          get_local $l3
          i32.const 5
          i32.lt_u
          select
          i32.gt_u
          br_if $B3
          get_local $l4
          set_local $l3
          get_local $l2
          call $get_bits_left
          i32.const 0
          i32.lt_s
          br_if $B2
          block $B10
            block $B11
              get_local $p0
              i32.load offset=272
              tee_local $l3
              i32.eqz
              br_if $B11
              get_local $l3
              i32.load offset=4
              get_local $l8
              i32.load offset=4
              get_local $l8
              i32.load offset=8
              call $memcmp
              br_if $B11
              get_local $l10
              i32.const 12
              i32.add
              call $av_buffer_unref
              br $B10
            end
            get_local $p0
            i32.const 272
            i32.add
            set_local $l3
            i32.const 0
            set_local $l4
            loop $L12
              block $B13
                get_local $p0
                get_local $l4
                i32.const 2
                i32.shl
                i32.add
                i32.const 400
                i32.add
                tee_local $l0
                i32.load
                tee_local $l2
                i32.eqz
                br_if $B13
                get_local $l2
                i32.load offset=4
                i32.load
                br_if $B13
                get_local $l0
                call $av_buffer_unref
              end
              get_local $l4
              i32.const 1
              i32.add
              tee_local $l4
              i32.const 256
              i32.ne
              br_if $L12
            end
            block $B14
              get_local $l3
              i32.load
              tee_local $l4
              i32.eqz
              br_if $B14
              get_local $p0
              i32.load offset=200
              get_local $l4
              i32.load offset=4
              i32.ne
              br_if $B14
              get_local $p0
              i32.const 1424
              i32.add
              call $av_buffer_unref
              get_local $p0
              get_local $p0
              i32.load offset=272
              call $av_buffer_ref
              tee_local $l4
              i32.store offset=1424
              get_local $l4
              br_if $B14
              get_local $p0
              i32.const 0
              i32.store offset=200
            end
            get_local $l3
            call $av_buffer_unref
            get_local $l3
            get_local $l8
            i32.store
          end
          i32.const 0
          set_local $l3
          br $B0
        end
        get_local $l4
        set_local $l3
      end
      get_local $l10
      i32.const 12
      i32.add
      call $av_buffer_unref
    end
    get_local $l10
    i32.const 16
    i32.add
    set_global $g0
    get_local $l3)
  (func $create_dummy_vps (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    i32.const 468
    call $av_buffer_allocz
    tee_local $l1
    i32.eqz
    if $I0
      i32.const -48
      return
    end
    get_local $l1
    i32.load offset=4
    tee_local $l0
    i64.const 4294967297
    i64.store offset=4 align=4
    get_local $l0
    i32.const 0
    i32.store8 offset=444
    get_local $l0
    i64.const 4294967296
    i64.store offset=436 align=4
    get_local $l0
    i32.const -1
    i32.store offset=408
    get_local $l0
    i32.const 0
    i32.store offset=380
    get_local $l0
    i64.const 4294967297
    i64.store offset=348 align=4
    get_local $l0
    i32.const 0
    i32.store8
    get_local $p0
    i32.const 208
    i32.add
    call $av_buffer_unref
    get_local $p0
    get_local $l1
    i32.store offset=208
    i32.const 0)
  (func $ff_hevc_decode_nal_pps (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32) (local $l13 i32) (local $l14 i32) (local $l15 i32) (local $l16 i32) (local $l17 i32) (local $l18 i32) (local $l19 i32) (local $l20 i32) (local $l21 i32) (local $l22 i32) (local $l23 i32) (local $l24 i64)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l15
    set_global $g0
    get_local $p0
    i32.load offset=136
    set_local $l0
    get_local $l15
    i32.const 1692
    call $av_mallocz
    tee_local $l1
    i32.store offset=8
    i32.const -48
    set_local $l8
    block $B0
      get_local $l1
      i32.eqz
      br_if $B0
      get_local $l15
      get_local $l1
      i32.const 1692
      i32.const 29
      call $av_buffer_create
      tee_local $l18
      i32.store offset=12
      get_local $l18
      i32.eqz
      if $I1
        get_local $l15
        i32.const 8
        i32.add
        call $av_freep
        br $B0
      end
      get_local $l1
      i32.const 2
      i32.store8 offset=1629
      get_local $l1
      i64.const 0
      i64.store offset=60 align=4
      get_local $l1
      i32.const 0
      i32.store8 offset=57
      get_local $l1
      i64.const 4294967297
      i64.store offset=44 align=4
      get_local $l1
      i32.const 257
      i32.store16 offset=52
      block $B2
        block $B3
          get_local $l0
          i32.const 204
          i32.add
          tee_local $l3
          call $get_ue_golomb_long
          tee_local $l23
          i32.const 255
          i32.gt_u
          br_if $B3
          get_local $l1
          get_local $l3
          call $get_ue_golomb_long
          tee_local $l0
          i32.store
          get_local $l0
          i32.const 31
          i32.gt_u
          br_if $B3
          get_local $p0
          get_local $l0
          i32.const 2
          i32.shl
          i32.add
          i32.load offset=272
          tee_local $l0
          i32.eqz
          br_if $B3
          get_local $l0
          i32.load offset=4
          set_local $l7
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=41
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=39
          get_local $l1
          get_local $l3
          i32.const 3
          call $get_bits
          i32.store offset=1624
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=4
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=5
          get_local $l1
          get_local $l3
          call $get_ue_golomb_long
          i32.const 1
          i32.add
          i32.store offset=8
          get_local $l1
          get_local $l3
          call $get_ue_golomb_long
          i32.const 1
          i32.add
          i32.store offset=12
          get_local $l1
          get_local $l3
          call $get_se_golomb_long
          i32.store offset=16
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=20
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=21
          get_local $l3
          call $get_bits1
          set_local $l0
          get_local $l1
          i32.const 0
          i32.store offset=24
          get_local $l1
          get_local $l0
          i32.store8 offset=22
          get_local $l0
          i32.const 255
          i32.and
          if $I4
            get_local $l1
            get_local $l3
            call $get_ue_golomb_long
            i32.store offset=24
          end
          get_local $l1
          get_local $l3
          call $get_se_golomb_long
          tee_local $l0
          i32.store offset=28
          get_local $l0
          i32.const 12
          i32.add
          i32.const 24
          i32.gt_u
          br_if $B3
          get_local $l1
          get_local $l3
          call $get_se_golomb_long
          tee_local $l0
          i32.store offset=32
          get_local $l0
          i32.const 12
          i32.add
          i32.const 24
          i32.gt_u
          br_if $B3
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=36
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=37
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=38
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=40
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=42
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=43
          get_local $l1
          i32.load8_u offset=42
          if $I5
            get_local $l1
            get_local $l3
            call $get_ue_golomb_long
            i32.const 1
            i32.add
            i32.store offset=44
            get_local $l1
            get_local $l3
            call $get_ue_golomb_long
            i32.const 1
            i32.add
            tee_local $l4
            i32.store offset=48
            get_local $l1
            i32.load offset=44
            tee_local $l0
            i32.eqz
            get_local $l4
            i32.eqz
            i32.or
            br_if $B3
            get_local $l0
            get_local $l7
            i32.load offset=13120
            i32.ge_s
            br_if $B3
            get_local $l4
            get_local $l7
            i32.load offset=13124
            i32.ge_s
            br_if $B3
            get_local $l1
            get_local $l0
            i32.const 4
            call $av_malloc_array
            i32.store offset=1648
            get_local $l1
            get_local $l1
            i32.load offset=48
            i32.const 4
            call $av_malloc_array
            tee_local $l0
            i32.store offset=1652
            get_local $l1
            i32.load offset=1648
            i32.eqz
            get_local $l0
            i32.eqz
            i32.or
            br_if $B2
            get_local $l1
            get_local $l3
            call $get_bits1
            tee_local $l0
            i32.store8 offset=52
            get_local $l0
            i32.const 255
            i32.and
            i32.eqz
            if $I6
              block $B7
                get_local $l1
                i32.load offset=44
                tee_local $l0
                i32.const 2
                i32.lt_s
                if $I8
                  get_local $l0
                  i32.const 1
                  i32.sub
                  set_local $l2
                  br $B7
                end
                i32.const 0
                set_local $l0
                loop $L9
                  get_local $l3
                  call $get_ue_golomb_long
                  set_local $l4
                  get_local $l1
                  i32.load offset=1648
                  get_local $l0
                  i32.const 2
                  i32.shl
                  i32.add
                  get_local $l4
                  i32.const 1
                  i32.add
                  tee_local $l4
                  i32.store
                  get_local $l24
                  get_local $l4
                  i64.extend_u/i32
                  i64.add
                  set_local $l24
                  get_local $l0
                  i32.const 1
                  i32.add
                  tee_local $l0
                  get_local $l1
                  i32.load offset=44
                  i32.const 1
                  i32.sub
                  tee_local $l2
                  i32.lt_s
                  br_if $L9
                end
              end
              get_local $l24
              get_local $l7
              i32.load offset=13128
              tee_local $l0
              i64.extend_s/i32
              i64.ge_u
              br_if $B3
              get_local $l1
              i32.load offset=1648
              get_local $l2
              i32.const 2
              i32.shl
              i32.add
              get_local $l0
              get_local $l24
              i32.wrap/i64
              i32.sub
              i32.store
              i64.const 0
              set_local $l24
              block $B10
                get_local $l1
                i32.load offset=48
                tee_local $l0
                i32.const 2
                i32.lt_s
                if $I11
                  get_local $l0
                  i32.const 1
                  i32.sub
                  set_local $l2
                  br $B10
                end
                i32.const 0
                set_local $l0
                loop $L12
                  get_local $l3
                  call $get_ue_golomb_long
                  set_local $l4
                  get_local $l1
                  i32.load offset=1652
                  get_local $l0
                  i32.const 2
                  i32.shl
                  i32.add
                  get_local $l4
                  i32.const 1
                  i32.add
                  tee_local $l4
                  i32.store
                  get_local $l24
                  get_local $l4
                  i64.extend_u/i32
                  i64.add
                  set_local $l24
                  get_local $l0
                  i32.const 1
                  i32.add
                  tee_local $l0
                  get_local $l1
                  i32.load offset=48
                  i32.const 1
                  i32.sub
                  tee_local $l2
                  i32.lt_s
                  br_if $L12
                end
              end
              get_local $l24
              get_local $l7
              i32.load offset=13132
              tee_local $l0
              i64.extend_s/i32
              i64.ge_u
              br_if $B3
              get_local $l1
              i32.load offset=1652
              get_local $l2
              i32.const 2
              i32.shl
              i32.add
              get_local $l0
              get_local $l24
              i32.wrap/i64
              i32.sub
              i32.store
            end
            get_local $l1
            get_local $l3
            call $get_bits1
            i32.store8 offset=53
          end
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=54
          get_local $l1
          get_local $l3
          call $get_bits1
          tee_local $l0
          i32.store8 offset=55
          block $B13
            get_local $l0
            i32.const 255
            i32.and
            i32.eqz
            br_if $B13
            get_local $l1
            get_local $l3
            call $get_bits1
            i32.store8 offset=56
            get_local $l1
            get_local $l3
            call $get_bits1
            tee_local $l0
            i32.store8 offset=57
            get_local $l0
            i32.const 255
            i32.and
            br_if $B13
            get_local $l1
            get_local $l3
            call $get_se_golomb_long
            i32.const 1
            i32.shl
            i32.store offset=60
            get_local $l1
            get_local $l3
            call $get_se_golomb_long
            tee_local $l0
            i32.const 1
            i32.shl
            i32.store offset=64
            get_local $l0
            i32.const 6
            i32.add
            i32.const 12
            i32.gt_u
            br_if $B3
            get_local $l1
            i32.load offset=60
            i32.const 13
            i32.add
            i32.const 26
            i32.gt_u
            br_if $B3
          end
          get_local $l1
          get_local $l3
          call $get_bits1
          tee_local $l0
          i32.store8 offset=68
          get_local $l0
          i32.const 255
          i32.and
          if $I14
            get_local $l1
            i32.const 69
            i32.add
            tee_local $l0
            call $set_default_scaling_list_data
            get_local $p0
            get_local $l0
            get_local $l7
            call $scaling_list_data
            tee_local $l19
            set_local $l8
            get_local $l19
            i32.const 0
            i32.lt_s
            br_if $B2
          end
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=1617
          get_local $l1
          get_local $l3
          call $get_ue_golomb_long
          i32.const 2
          i32.add
          tee_local $l0
          i32.store offset=1620
          i32.const -1094995529
          set_local $l8
          get_local $l0
          get_local $l7
          i32.load offset=13080
          i32.gt_u
          br_if $B2
          get_local $l1
          get_local $l3
          call $get_bits1
          i32.store8 offset=1628
          block $B15
            get_local $l3
            call $get_bits1
            i32.eqz
            br_if $B15
            get_local $l3
            call $get_bits1
            set_local $l0
            get_local $l3
            i32.const 7
            call $get_bits
            drop
            get_local $l0
            i32.eqz
            br_if $B15
            get_local $p0
            get_local $l1
            call $pps_range_extensions
          end
          get_local $l1
          get_local $l1
          i32.load offset=44
          i32.const 1
          i32.add
          i32.const 4
          call $av_malloc_array
          i32.store offset=1656
          get_local $l1
          get_local $l1
          i32.load offset=48
          i32.const 1
          i32.add
          i32.const 4
          call $av_malloc_array
          i32.store offset=1660
          get_local $l1
          get_local $l7
          i32.load offset=13128
          i32.const 4
          call $av_malloc_array
          tee_local $l0
          i32.store offset=1664
          i32.const -48
          set_local $l8
          get_local $l1
          i32.load offset=1656
          i32.eqz
          br_if $B2
          get_local $l1
          i32.load offset=1660
          i32.eqz
          get_local $l0
          i32.eqz
          i32.or
          br_if $B2
          block $B16
            get_local $l1
            i32.load8_u offset=52
            i32.eqz
            br_if $B16
            get_local $l1
            i32.load offset=1648
            tee_local $l6
            i32.eqz
            if $I17
              get_local $l1
              get_local $l1
              i32.load offset=44
              i32.const 4
              call $av_malloc_array
              i32.store offset=1648
              get_local $l1
              get_local $l1
              i32.load offset=48
              i32.const 4
              call $av_malloc_array
              i32.store offset=1652
              get_local $l1
              i32.load offset=1648
              tee_local $l6
              i32.eqz
              br_if $B2
            end
            get_local $l1
            i32.load offset=1652
            tee_local $l11
            i32.eqz
            br_if $B2
            i32.const 0
            set_local $l0
            get_local $l1
            i32.load offset=44
            tee_local $l2
            i32.const 0
            i32.gt_s
            if $I18
              loop $L19
                get_local $l6
                get_local $l0
                i32.const 2
                i32.shl
                i32.add
                get_local $l7
                i32.load offset=13128
                tee_local $l5
                get_local $l0
                i32.const 1
                i32.add
                tee_local $l4
                i32.mul
                get_local $l2
                i32.div_s
                get_local $l0
                get_local $l5
                i32.mul
                get_local $l2
                i32.div_s
                i32.sub
                i32.store
                get_local $l1
                i32.load offset=44
                tee_local $l2
                get_local $l4
                tee_local $l0
                i32.gt_s
                br_if $L19
              end
            end
            get_local $l1
            i32.load offset=48
            tee_local $l2
            i32.const 1
            i32.lt_s
            br_if $B16
            i32.const 0
            set_local $l0
            loop $L20
              get_local $l11
              get_local $l0
              i32.const 2
              i32.shl
              i32.add
              get_local $l7
              i32.load offset=13132
              tee_local $l6
              get_local $l0
              i32.const 1
              i32.add
              tee_local $l4
              i32.mul
              get_local $l2
              i32.div_s
              get_local $l0
              get_local $l6
              i32.mul
              get_local $l2
              i32.div_s
              i32.sub
              i32.store
              get_local $l1
              i32.load offset=48
              tee_local $l2
              get_local $l4
              tee_local $l0
              i32.gt_s
              br_if $L20
            end
          end
          get_local $l1
          i32.load offset=1656
          tee_local $l11
          i32.const 0
          i32.store
          get_local $l1
          i32.load offset=44
          i32.const 1
          i32.ge_s
          if $I21
            get_local $l1
            i32.load offset=1648
            set_local $l6
            i32.const 0
            set_local $l5
            i32.const 0
            set_local $l0
            loop $L22
              get_local $l11
              get_local $l0
              i32.const 1
              i32.add
              tee_local $l4
              i32.const 2
              i32.shl
              i32.add
              get_local $l6
              get_local $l0
              i32.const 2
              i32.shl
              i32.add
              i32.load
              get_local $l5
              i32.add
              tee_local $l5
              i32.store
              get_local $l4
              tee_local $l0
              get_local $l1
              i32.load offset=44
              i32.lt_s
              br_if $L22
            end
          end
          get_local $l1
          i32.load offset=1660
          tee_local $l2
          i32.const 0
          i32.store
          get_local $l1
          i32.load offset=48
          i32.const 1
          i32.ge_s
          if $I23
            get_local $l1
            i32.load offset=1652
            set_local $l6
            i32.const 0
            set_local $l5
            i32.const 0
            set_local $l0
            loop $L24
              get_local $l2
              get_local $l0
              i32.const 1
              i32.add
              tee_local $l4
              i32.const 2
              i32.shl
              i32.add
              get_local $l6
              get_local $l0
              i32.const 2
              i32.shl
              i32.add
              i32.load
              get_local $l5
              i32.add
              tee_local $l5
              i32.store
              get_local $l4
              tee_local $l0
              get_local $l1
              i32.load offset=48
              i32.lt_s
              br_if $L24
            end
          end
          get_local $l7
          i32.load offset=13128
          tee_local $l5
          i32.const 1
          i32.ge_s
          if $I25
            get_local $l1
            i32.load offset=1664
            set_local $l4
            i32.const 0
            set_local $l0
            i32.const 0
            set_local $l2
            loop $L26
              get_local $l4
              get_local $l0
              i32.const 2
              i32.shl
              i32.add
              get_local $l2
              get_local $l0
              get_local $l11
              get_local $l2
              i32.const 2
              i32.shl
              i32.add
              i32.load
              i32.gt_u
              i32.add
              tee_local $l2
              i32.store
              get_local $l0
              i32.const 1
              i32.add
              tee_local $l0
              get_local $l7
              i32.load offset=13128
              tee_local $l5
              i32.lt_s
              br_if $L26
            end
          end
          get_local $l1
          get_local $l7
          i32.load offset=13132
          get_local $l5
          i32.mul
          tee_local $l17
          i32.const 4
          call $av_malloc_array
          i32.store offset=1668
          get_local $l1
          get_local $l17
          i32.const 4
          call $av_malloc_array
          i32.store offset=1672
          get_local $l1
          get_local $l17
          i32.const 4
          call $av_malloc_array
          i32.store offset=1676
          get_local $l1
          get_local $l7
          i32.load offset=13164
          i32.const 2
          i32.add
          tee_local $l0
          get_local $l0
          i32.mul
          i32.const 4
          call $av_malloc_array
          tee_local $l0
          i32.store offset=1688
          get_local $l1
          i32.load offset=1668
          tee_local $l20
          i32.eqz
          br_if $B2
          get_local $l1
          i32.load offset=1672
          tee_local $l11
          i32.eqz
          br_if $B2
          get_local $l1
          i32.load offset=1676
          tee_local $l16
          i32.eqz
          get_local $l0
          i32.eqz
          i32.or
          br_if $B2
          get_local $l17
          i32.const 1
          i32.ge_s
          if $I27
            get_local $l1
            i32.load offset=1656
            set_local $l21
            get_local $l1
            i32.load offset=1648
            set_local $l22
            get_local $l1
            i32.load offset=1660
            set_local $l12
            loop $L28
              get_local $l1
              i32.load offset=44
              tee_local $l0
              i32.const 0
              get_local $l0
              i32.const 0
              i32.gt_s
              select
              set_local $l4
              get_local $l9
              get_local $l9
              get_local $l7
              i32.load offset=13128
              tee_local $l10
              i32.div_s
              tee_local $l13
              get_local $l10
              i32.mul
              i32.sub
              set_local $l14
              i32.const 0
              set_local $l0
              loop $L29
                block $B30
                  i32.const 0
                  set_local $l2
                  get_local $l4
                  get_local $l0
                  tee_local $l5
                  i32.eq
                  if $I31
                    i32.const 0
                    set_local $l5
                    br $B30
                  end
                  get_local $l14
                  get_local $l21
                  get_local $l5
                  i32.const 1
                  i32.add
                  tee_local $l0
                  i32.const 2
                  i32.shl
                  i32.add
                  i32.load
                  i32.ge_u
                  br_if $L29
                end
              end
              get_local $l1
              i32.load offset=48
              tee_local $l0
              i32.const 0
              get_local $l0
              i32.const 0
              i32.gt_s
              select
              set_local $l4
              loop $L32
                block $B33
                  i32.const 0
                  set_local $l0
                  get_local $l4
                  get_local $l2
                  tee_local $l6
                  i32.eq
                  if $I34
                    i32.const 0
                    set_local $l6
                    br $B33
                  end
                  get_local $l13
                  get_local $l12
                  get_local $l6
                  i32.const 1
                  i32.add
                  tee_local $l2
                  i32.const 2
                  i32.shl
                  i32.add
                  i32.load
                  i32.ge_u
                  br_if $L32
                end
              end
              get_local $l5
              if $I35
                get_local $l1
                i32.load offset=1652
                get_local $l6
                i32.const 2
                i32.shl
                i32.add
                i32.load
                set_local $l4
                i32.const 0
                set_local $l2
                loop $L36
                  get_local $l22
                  get_local $l2
                  i32.const 2
                  i32.shl
                  i32.add
                  i32.load
                  get_local $l4
                  i32.mul
                  get_local $l0
                  i32.add
                  set_local $l0
                  get_local $l2
                  i32.const 1
                  i32.add
                  tee_local $l2
                  get_local $l5
                  i32.ne
                  br_if $L36
                end
              end
              get_local $l6
              if $I37
                get_local $l1
                i32.load offset=1652
                set_local $l4
                i32.const 0
                set_local $l2
                loop $L38
                  get_local $l4
                  get_local $l2
                  i32.const 2
                  i32.shl
                  i32.add
                  i32.load
                  get_local $l10
                  i32.mul
                  get_local $l0
                  i32.add
                  set_local $l0
                  get_local $l2
                  i32.const 1
                  i32.add
                  tee_local $l2
                  get_local $l6
                  i32.ne
                  br_if $L38
                end
              end
              get_local $l20
              get_local $l9
              i32.const 2
              i32.shl
              i32.add
              get_local $l0
              get_local $l14
              i32.add
              get_local $l22
              get_local $l5
              i32.const 2
              i32.shl
              tee_local $l0
              i32.add
              i32.load
              get_local $l13
              get_local $l12
              get_local $l6
              i32.const 2
              i32.shl
              i32.add
              i32.load
              i32.sub
              i32.mul
              i32.add
              get_local $l0
              get_local $l21
              i32.add
              i32.load
              i32.sub
              tee_local $l0
              i32.store
              get_local $l11
              get_local $l0
              i32.const 2
              i32.shl
              i32.add
              get_local $l9
              i32.store
              get_local $l9
              i32.const 1
              i32.add
              tee_local $l9
              get_local $l17
              i32.ne
              br_if $L28
            end
          end
          block $B39
            get_local $l1
            i32.load offset=48
            tee_local $l2
            i32.const 1
            i32.lt_s
            if $I40
              i32.const 0
              set_local $l5
              br $B39
            end
            get_local $l1
            i32.load offset=44
            set_local $l0
            i32.const 0
            set_local $l9
            i32.const 0
            set_local $l5
            loop $L41
              block $B42
                get_local $l0
                i32.const 0
                i32.le_s
                if $I43
                  get_local $l9
                  i32.const 1
                  i32.add
                  set_local $l9
                  br $B42
                end
                get_local $l1
                i32.load offset=1660
                tee_local $l4
                get_local $l9
                i32.const 2
                i32.shl
                i32.add
                set_local $l13
                get_local $l4
                get_local $l9
                i32.const 1
                i32.add
                tee_local $l9
                i32.const 2
                i32.shl
                i32.add
                tee_local $l14
                i32.load
                set_local $l10
                i32.const 0
                set_local $l12
                loop $L44
                  block $B45
                    get_local $l10
                    get_local $l13
                    i32.load
                    tee_local $l2
                    i32.le_u
                    if $I46
                      get_local $l12
                      i32.const 1
                      i32.add
                      set_local $l12
                      br $B45
                    end
                    get_local $l1
                    i32.load offset=1656
                    tee_local $l0
                    get_local $l12
                    i32.const 2
                    i32.shl
                    i32.add
                    set_local $l11
                    get_local $l0
                    get_local $l12
                    i32.const 1
                    i32.add
                    tee_local $l12
                    i32.const 2
                    i32.shl
                    i32.add
                    tee_local $l4
                    i32.load
                    set_local $l6
                    loop $L47
                      get_local $l6
                      get_local $l11
                      i32.load
                      tee_local $l0
                      i32.gt_u
                      if $I48
                        loop $L49
                          get_local $l16
                          get_local $l20
                          get_local $l7
                          i32.load offset=13128
                          get_local $l2
                          i32.mul
                          get_local $l0
                          i32.add
                          i32.const 2
                          i32.shl
                          i32.add
                          i32.load
                          i32.const 2
                          i32.shl
                          i32.add
                          get_local $l5
                          i32.store
                          get_local $l0
                          i32.const 1
                          i32.add
                          tee_local $l0
                          get_local $l4
                          i32.load
                          tee_local $l6
                          i32.lt_u
                          br_if $L49
                        end
                        get_local $l14
                        i32.load
                        set_local $l10
                      end
                      get_local $l2
                      i32.const 1
                      i32.add
                      tee_local $l2
                      get_local $l10
                      i32.lt_u
                      br_if $L47
                    end
                    get_local $l1
                    i32.load offset=44
                    set_local $l0
                  end
                  get_local $l5
                  i32.const 1
                  i32.add
                  set_local $l5
                  get_local $l0
                  get_local $l12
                  i32.gt_s
                  br_if $L44
                end
                get_local $l1
                i32.load offset=48
                set_local $l2
              end
              get_local $l2
              get_local $l9
              i32.gt_s
              br_if $L41
            end
          end
          get_local $l1
          get_local $l5
          i32.const 4
          call $av_malloc_array
          tee_local $l5
          i32.store offset=1680
          get_local $l5
          i32.eqz
          br_if $B2
          get_local $l1
          i32.load offset=48
          tee_local $l0
          i32.const 1
          i32.ge_s
          if $I50
            get_local $l1
            i32.load offset=44
            set_local $l2
            i32.const 0
            set_local $l6
            loop $L51
              get_local $l2
              i32.const 1
              i32.ge_s
              if $I52
                get_local $l1
                i32.load offset=1660
                get_local $l6
                i32.const 2
                i32.shl
                i32.add
                set_local $l8
                get_local $l1
                i32.load offset=1656
                set_local $l4
                i32.const 0
                set_local $l0
                loop $L53
                  get_local $l5
                  get_local $l2
                  get_local $l6
                  i32.mul
                  get_local $l0
                  i32.add
                  i32.const 2
                  i32.shl
                  i32.add
                  get_local $l4
                  get_local $l0
                  i32.const 2
                  i32.shl
                  i32.add
                  i32.load
                  get_local $l7
                  i32.load offset=13128
                  get_local $l8
                  i32.load
                  i32.mul
                  i32.add
                  i32.store
                  get_local $l0
                  i32.const 1
                  i32.add
                  tee_local $l0
                  get_local $l1
                  i32.load offset=44
                  tee_local $l2
                  i32.lt_s
                  br_if $L53
                end
                get_local $l1
                i32.load offset=48
                set_local $l0
              end
              get_local $l6
              i32.const 1
              i32.add
              tee_local $l6
              get_local $l0
              i32.lt_s
              br_if $L51
            end
          end
          get_local $l7
          i32.load offset=13072
          set_local $l5
          get_local $l7
          i32.load offset=13080
          set_local $l4
          get_local $l1
          get_local $l1
          i32.load offset=1688
          tee_local $l2
          get_local $l7
          i32.load offset=13164
          tee_local $l0
          i32.const 2
          i32.shl
          i32.add
          i32.const 12
          i32.add
          tee_local $l13
          i32.store offset=1684
          block $B54
            get_local $l0
            i32.const -1
            i32.lt_s
            br_if $B54
            get_local $l0
            i32.const 2
            i32.add
            set_local $l6
            i32.const 0
            set_local $l0
            loop $L55
              get_local $l2
              get_local $l0
              get_local $l6
              i32.mul
              i32.const 2
              i32.shl
              i32.add
              i32.const -1
              i32.store
              get_local $l2
              get_local $l0
              i32.const 2
              i32.shl
              i32.add
              i32.const -1
              i32.store
              get_local $l0
              i32.const 1
              i32.add
              tee_local $l0
              get_local $l7
              i32.load offset=13164
              tee_local $l8
              i32.const 2
              i32.add
              tee_local $l6
              i32.lt_s
              br_if $L55
            end
            get_local $l8
            i32.const 0
            i32.lt_s
            br_if $B54
            get_local $l4
            get_local $l5
            i32.sub
            tee_local $l10
            i32.const 1
            i32.shl
            set_local $l14
            i32.const 0
            set_local $l2
            loop $L56
              get_local $l8
              i32.const 0
              i32.ge_s
              if $I57
                get_local $l2
                get_local $l10
                i32.shr_u
                set_local $l11
                get_local $l1
                i32.load offset=1668
                set_local $l4
                i32.const 0
                set_local $l5
                loop $L58
                  get_local $l4
                  get_local $l7
                  i32.load offset=13128
                  get_local $l11
                  i32.mul
                  get_local $l5
                  get_local $l10
                  i32.shr_u
                  i32.add
                  i32.const 2
                  i32.shl
                  i32.add
                  i32.load
                  get_local $l14
                  i32.shl
                  set_local $l6
                  i32.const 0
                  set_local $l0
                  get_local $l10
                  i32.const 1
                  i32.ge_s
                  if $I59
                    loop $L60
                      get_local $l6
                      i32.const 1
                      get_local $l0
                      i32.shl
                      tee_local $l16
                      i32.const 1
                      i32.shl
                      get_local $l0
                      i32.shl
                      i32.const 0
                      get_local $l2
                      get_local $l16
                      i32.and
                      select
                      i32.add
                      get_local $l16
                      get_local $l0
                      i32.shl
                      i32.const 0
                      get_local $l5
                      get_local $l16
                      i32.and
                      select
                      i32.add
                      set_local $l6
                      get_local $l0
                      i32.const 1
                      i32.add
                      tee_local $l0
                      get_local $l10
                      i32.ne
                      br_if $L60
                    end
                  end
                  get_local $l13
                  get_local $l8
                  i32.const 2
                  i32.add
                  get_local $l2
                  i32.mul
                  get_local $l5
                  i32.add
                  i32.const 2
                  i32.shl
                  i32.add
                  get_local $l6
                  i32.store
                  get_local $l5
                  get_local $l7
                  i32.load offset=13164
                  tee_local $l8
                  i32.lt_s
                  set_local $l0
                  get_local $l5
                  i32.const 1
                  i32.add
                  set_local $l5
                  get_local $l0
                  br_if $L58
                end
              end
              get_local $l2
              get_local $l8
              i32.lt_s
              set_local $l0
              get_local $l2
              i32.const 1
              i32.add
              set_local $l2
              get_local $l0
              br_if $L56
            end
          end
          get_local $l19
          set_local $l8
          get_local $l3
          call $get_bits_left
          i32.const 0
          i32.lt_s
          br_if $B2
          get_local $p0
          get_local $l23
          i32.const 2
          i32.shl
          i32.add
          tee_local $p0
          i32.const 400
          i32.add
          call $av_buffer_unref
          get_local $p0
          get_local $l18
          i32.store offset=400
          i32.const 0
          set_local $l8
          br $B0
        end
        i32.const -1094995529
        set_local $l8
      end
      get_local $l15
      i32.const 12
      i32.add
      call $av_buffer_unref
    end
    get_local $l15
    i32.const 16
    i32.add
    set_global $g0
    get_local $l8)
  (func $hevc_pps_free (type $t4) (param $p0 i32) (param $p1 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $p0
    set_global $g0
    get_local $p0
    get_local $p1
    i32.store offset=12
    get_local $p1
    i32.const 1648
    i32.add
    call $av_freep
    get_local $p1
    i32.const 1652
    i32.add
    call $av_freep
    get_local $p1
    i32.const 1656
    i32.add
    call $av_freep
    get_local $p1
    i32.const 1660
    i32.add
    call $av_freep
    get_local $p1
    i32.const 1664
    i32.add
    call $av_freep
    get_local $p1
    i32.const 1668
    i32.add
    call $av_freep
    get_local $p1
    i32.const 1672
    i32.add
    call $av_freep
    get_local $p1
    i32.const 1680
    i32.add
    call $av_freep
    get_local $p1
    i32.const 1676
    i32.add
    call $av_freep
    get_local $p1
    i32.const 1688
    i32.add
    call $av_freep
    get_local $p0
    i32.const 12
    i32.add
    call $av_freep
    get_local $p0
    i32.const 16
    i32.add
    set_global $g0)
  (func $set_default_scaling_list_data (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32)
    loop $L0
      get_local $p0
      get_local $l0
      i32.const 6
      i32.shl
      i32.add
      tee_local $l1
      i64.const 1157442765409226768
      i64.store align=1
      get_local $l1
      i64.const 1157442765409226768
      i64.store offset=8 align=1
      get_local $p0
      get_local $l0
      i32.add
      tee_local $l1
      i32.const 1542
      i32.add
      i32.const 16
      i32.store8
      get_local $l1
      i32.const 1536
      i32.add
      i32.const 16
      i32.store8
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 6
      i32.ne
      br_if $L0
    end
    get_local $p0
    i32.const 384
    i32.add
    i32.const 2672
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 448
    i32.add
    i32.const 2672
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 512
    i32.add
    i32.const 2672
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 576
    i32.add
    i32.const 2736
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 640
    i32.add
    i32.const 2736
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 704
    i32.add
    i32.const 2736
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 768
    i32.add
    i32.const 2672
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 832
    i32.add
    i32.const 2672
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 896
    i32.add
    i32.const 2672
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 960
    i32.add
    i32.const 2736
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 1024
    i32.add
    i32.const 2736
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 1088
    i32.add
    i32.const 2736
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 1152
    i32.add
    i32.const 2672
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 1216
    i32.add
    i32.const 2672
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 1280
    i32.add
    i32.const 2672
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 1344
    i32.add
    i32.const 2736
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 1408
    i32.add
    i32.const 2736
    i32.const 64
    call $memcpy
    drop
    get_local $p0
    i32.const 1472
    i32.add
    i32.const 2736
    i32.const 64
    call $memcpy
    drop)
  (func $scaling_list_data (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32)
    get_local $p0
    i32.load offset=136
    i32.const 204
    i32.add
    set_local $l3
    loop $L0
      i32.const 1
      get_local $l0
      i32.const 1
      i32.shl
      i32.const 4
      i32.add
      i32.shl
      tee_local $p0
      i32.const 64
      get_local $p0
      i32.const 64
      i32.lt_s
      select
      tee_local $p0
      i32.const 1
      get_local $p0
      i32.const 1
      i32.gt_s
      select
      set_local $l6
      i32.const 64
      i32.const 16
      get_local $l0
      select
      set_local $l7
      i32.const 3
      i32.const 1
      get_local $l0
      i32.const 3
      i32.eq
      select
      set_local $l8
      get_local $l0
      i32.const 2
      i32.sub
      set_local $l4
      i32.const 0
      set_local $l1
      loop $L1
        block $B2
          get_local $l3
          call $get_bits1
          i32.const 255
          i32.and
          i32.eqz
          if $I3
            get_local $l3
            call $get_ue_golomb_long
            tee_local $p0
            i32.eqz
            br_if $B2
            get_local $p0
            get_local $l1
            i32.gt_u
            if $I4
              i32.const -1094995529
              return
            end
            get_local $p1
            get_local $l0
            i32.const 384
            i32.mul
            i32.add
            tee_local $l2
            get_local $l1
            i32.const 6
            i32.shl
            i32.add
            get_local $l2
            get_local $l1
            get_local $p0
            i32.sub
            tee_local $p0
            i32.const 6
            i32.shl
            i32.add
            get_local $l7
            call $memcpy
            drop
            get_local $l0
            i32.const 2
            i32.lt_u
            br_if $B2
            get_local $p1
            get_local $l4
            i32.const 6
            i32.mul
            i32.add
            i32.const 1536
            i32.add
            tee_local $l2
            get_local $l1
            i32.add
            get_local $p0
            get_local $l2
            i32.add
            i32.load8_u
            i32.store8
            br $B2
          end
          i32.const 8
          set_local $l2
          get_local $l0
          i32.const 2
          i32.ge_u
          if $I5
            get_local $p1
            get_local $l4
            i32.const 6
            i32.mul
            i32.add
            get_local $l1
            i32.add
            i32.const 1536
            i32.add
            get_local $l3
            call $get_se_golomb_long
            i32.const 8
            i32.add
            tee_local $l2
            i32.store8
          end
          i32.const 0
          set_local $p0
          loop $L6
            get_local $p1
            get_local $l0
            i32.const 384
            i32.mul
            i32.add
            get_local $l1
            i32.const 6
            i32.shl
            i32.add
            block $B7 (result i32)
              get_local $l0
              i32.eqz
              if $I8
                get_local $p0
                i32.const 1040
                i32.add
                i32.load8_u
                i32.const 2
                i32.shl
                set_local $l5
                get_local $p0
                i32.const 1024
                i32.add
                br $B7
              end
              get_local $p0
              i32.const 1120
              i32.add
              i32.load8_u
              i32.const 3
              i32.shl
              set_local $l5
              get_local $p0
              i32.const 1056
              i32.add
            end
            i32.load8_u
            get_local $l5
            i32.add
            i32.add
            get_local $l3
            call $get_se_golomb_long
            get_local $l2
            i32.add
            i32.const 256
            i32.add
            i32.const 256
            i32.rem_s
            tee_local $l2
            i32.store8
            get_local $p0
            i32.const 1
            i32.add
            tee_local $p0
            get_local $l6
            i32.ne
            br_if $L6
          end
        end
        get_local $l1
        get_local $l8
        i32.add
        tee_local $l1
        i32.const 6
        i32.lt_u
        br_if $L1
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 4
      i32.ne
      br_if $L0
    end
    get_local $p2
    i32.load offset=4
    i32.const 3
    i32.eq
    if $I9
      i32.const 0
      set_local $l0
      loop $L10
        get_local $p1
        get_local $l0
        i32.add
        tee_local $p0
        i32.const 1216
        i32.add
        get_local $p0
        i32.load8_u offset=832
        i32.store8
        get_local $p0
        i32.const 1280
        i32.add
        get_local $p0
        i32.load8_u offset=896
        i32.store8
        get_local $p0
        i32.const 1408
        i32.add
        get_local $p0
        i32.const 1024
        i32.add
        i32.load8_u
        i32.store8
        get_local $p0
        i32.const 1472
        i32.add
        get_local $p0
        i32.const 1088
        i32.add
        i32.load8_u
        i32.store8
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 64
        i32.ne
        br_if $L10
      end
      get_local $p1
      i32.const 1543
      i32.add
      get_local $p1
      i32.const 1537
      i32.add
      i32.load16_u align=1
      i32.store16 align=1
      get_local $p1
      i32.const 1546
      i32.add
      get_local $p1
      i32.const 1540
      i32.add
      i32.load16_u align=1
      i32.store16 align=1
    end
    i32.const 0)
  (func $pps_range_extensions (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load offset=136
    i32.const 204
    i32.add
    set_local $l0
    get_local $p1
    i32.load8_u offset=21
    if $I0
      get_local $p1
      get_local $l0
      call $get_ue_golomb_long
      i32.const 2
      i32.add
      i32.store8 offset=1629
    end
    get_local $p1
    get_local $l0
    call $get_bits1
    i32.store8 offset=1630
    get_local $p1
    get_local $l0
    call $get_bits1
    tee_local $p0
    i32.store8 offset=1631
    block $B1
      get_local $p0
      i32.const 255
      i32.and
      if $I2
        get_local $p1
        get_local $l0
        call $get_ue_golomb_long
        i32.store8 offset=1632
        get_local $p1
        get_local $l0
        call $get_ue_golomb_long
        tee_local $p0
        i32.store8 offset=1633
        get_local $p0
        i32.const 255
        i32.and
        i32.const 4
        i32.gt_u
        br_if $B1
        i32.const 0
        set_local $p0
        loop $L3
          get_local $p0
          get_local $p1
          i32.add
          tee_local $l1
          i32.const 1634
          i32.add
          get_local $l0
          call $get_se_golomb_long
          i32.store8
          get_local $l1
          i32.const 1639
          i32.add
          get_local $l0
          call $get_se_golomb_long
          i32.store8
          get_local $p0
          get_local $p1
          i32.load8_u offset=1633
          i32.lt_u
          set_local $l1
          get_local $p0
          i32.const 1
          i32.add
          set_local $p0
          get_local $l1
          br_if $L3
        end
      end
      get_local $p1
      get_local $l0
      call $get_ue_golomb_long
      i32.store8 offset=1644
      get_local $p1
      get_local $l0
      call $get_ue_golomb_long
      i32.store8 offset=1645
    end)
  (func $decode_nal_sei_message (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p0
    i32.load offset=136
    i32.const 204
    i32.add
    set_local $l0
    loop $L0
      get_local $l0
      i32.const 8
      call $get_bits
      tee_local $l3
      get_local $l2
      i32.add
      set_local $l2
      get_local $l3
      i32.const 255
      i32.eq
      br_if $L0
    end
    loop $L1
      get_local $l0
      i32.const 8
      call $get_bits
      tee_local $l3
      get_local $l1
      i32.add
      set_local $l1
      get_local $l3
      i32.const 255
      i32.eq
      br_if $L1
    end
    block $B2
      block $B3
        get_local $p0
        i32.load offset=2512
        i32.const 39
        i32.eq
        if $I4
          block $B5
            block $B6
              get_local $l2
              i32.const 256
              i32.sub
              br_table $B3 $B6 $B5
            end
            get_local $p0
            get_local $l0
            i32.const 16
            call $get_bits
            i32.store16 offset=4524
            return
          end
          get_local $l0
          get_local $l1
          i32.const 3
          i32.shl
          call $skip_bits
          return
        end
        get_local $l2
        i32.const 132
        i32.ne
        br_if $B2
      end
      get_local $p0
      call $decode_nal_sei_decoded_picture_hash
      return
    end
    get_local $l0
    get_local $l1
    i32.const 3
    i32.shl
    call $skip_bits)
  (func $decode_nal_sei_decoded_picture_hash (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_local $p0
    i32.load offset=136
    i32.const 204
    i32.add
    tee_local $l1
    i32.const 8
    call $get_bits
    i32.const 255
    i32.and
    set_local $l3
    loop $L0
      block $B1
        block $B2
          block $B3
            block $B4
              get_local $l3
              br_table $B4 $B3 $B2 $B1
            end
            get_local $p0
            i32.const 1
            i32.store8 offset=4468
            i32.const 0
            set_local $l2
            loop $L5
              get_local $p0
              get_local $l0
              i32.const 4
              i32.shl
              i32.add
              get_local $l2
              i32.add
              i32.const 4420
              i32.add
              get_local $l1
              i32.const 8
              call $get_bits
              i32.store8
              get_local $l2
              i32.const 1
              i32.add
              tee_local $l2
              i32.const 16
              i32.ne
              br_if $L5
            end
            br $B1
          end
          get_local $l1
          i32.const 16
          call $skip_bits
          br $B1
        end
        get_local $l1
        i32.const 32
        call $skip_bits
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 3
      i32.ne
      br_if $L0
    end)
  (func $avcodec_open2 (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    block $B0
      i32.const 3688
      i32.load
      tee_local $l0
      i32.const 1
      i32.ge_s
      if $I1
        get_local $p0
        i32.load offset=60
        br_if $B0
        get_local $p0
        get_local $l0
        call $av_mallocz
        tee_local $l0
        i32.store offset=60
        get_local $l0
        br_if $B0
        i32.const -48
        return
      end
      get_local $p0
      i32.const 0
      i32.store offset=60
    end
    get_local $p0
    i64.const 0
    i64.store offset=912
    get_local $p0
    i32.const 1
    i32.store offset=800
    get_local $p0
    i32.const 0
    i32.store offset=424
    get_local $p0
    i32.const 3636
    i32.store offset=12
    get_local $p0
    i64.const -9223372036854775808
    i64.store offset=936
    get_local $p0
    i64.const -9223372036854775808
    i64.store offset=928
    get_local $p0
    i64.const 0
    i64.store offset=920
    get_local $p0
    i32.const 3712
    i32.load
    call_indirect (type $t0)
    tee_local $l0
    i32.const -1
    i32.le_s
    if $I2 (result i32)
      get_local $p0
      i32.const 60
      i32.add
      call $av_freep
      get_local $p0
      i32.const 0
      i32.store offset=12
      get_local $l0
    else
      i32.const 0
    end)
  (func $avcodec_close (type $t1) (param $p0 i32)
    (local $l0 i32)
    get_local $p0
    if $I0
      block $B1
        get_local $p0
        i32.load offset=12
        tee_local $l0
        i32.eqz
        br_if $B1
        get_local $l0
        i32.load offset=92
        tee_local $l0
        i32.eqz
        br_if $B1
        get_local $p0
        get_local $l0
        call_indirect (type $t0)
        drop
      end
      get_local $p0
      i32.const 0
      i32.store offset=796
      get_local $p0
      i32.const 60
      i32.add
      call $av_freep
      get_local $p0
      i32.const 0
      i32.store offset=808
      get_local $p0
      i32.const 0
      i32.store offset=12
    end)
  (func $avcodec_default_execute (type $t18) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p4
    i32.const 1
    i32.ge_s
    if $I0
      loop $L1
        get_local $p0
        get_local $p2
        get_local $p5
        get_local $l0
        i32.mul
        i32.add
        get_local $p1
        call_indirect (type $t2)
        set_local $l1
        get_local $p3
        if $I2
          get_local $p3
          get_local $l0
          i32.const 2
          i32.shl
          i32.add
          get_local $l1
          i32.store
        end
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p4
        i32.ne
        br_if $L1
      end
    end
    i32.const 0)
  (func $avcodec_default_execute2 (type $t13) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p4
    i32.const 1
    i32.ge_s
    if $I0
      loop $L1
        get_local $p0
        get_local $p2
        get_local $l0
        i32.const 0
        get_local $p1
        call_indirect (type $t9)
        set_local $l1
        get_local $p3
        if $I2
          get_local $p3
          get_local $l0
          i32.const 2
          i32.shl
          i32.add
          get_local $l1
          i32.store
        end
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p4
        i32.ne
        br_if $L1
      end
    end
    i32.const 0)
  (func $avcodec_default_get_buffer2 (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_local $p1
    i32.load offset=76
    call $av_pix_fmt_desc_get
    tee_local $p2
    i32.load8_u offset=4
    if $I0
      i32.const 0
      set_local $p0
      loop $L1
        get_local $p1
        i32.load offset=64
        get_local $p2
        get_local $p0
        i32.const 1
        i32.shl
        i32.add
        i32.load16_u offset=8
        i32.const 11
        i32.shr_u
        i32.const 15
        i32.and
        i32.const 8
        i32.add
        i32.const 3
        i32.shr_u
        i32.mul
        i32.const 31
        i32.add
        i32.const -32
        i32.and
        set_local $l0
        block $B2 (result i32)
          get_local $p0
          i32.const 1
          i32.sub
          i32.const 1
          i32.le_u
          if $I3
            get_local $p1
            get_local $p0
            i32.const 2
            i32.shl
            i32.add
            i32.const 0
            i32.const 0
            get_local $l0
            i32.sub
            get_local $p2
            i32.load8_u offset=5
            i32.shr_s
            i32.sub
            tee_local $l0
            i32.store offset=32
            i32.const 0
            i32.const 0
            get_local $p1
            i32.load offset=68
            i32.const 31
            i32.add
            i32.const -32
            i32.and
            i32.sub
            get_local $p2
            i32.load8_u offset=6
            i32.shr_s
            i32.sub
            br $B2
          end
          get_local $p1
          get_local $p0
          i32.const 2
          i32.shl
          i32.add
          get_local $l0
          i32.store offset=32
          get_local $p1
          i32.load offset=68
          i32.const 31
          i32.add
          i32.const -32
          i32.and
        end
        set_local $l1
        get_local $p1
        get_local $p0
        i32.const 2
        i32.shl
        i32.add
        tee_local $l2
        get_local $l0
        get_local $l1
        i32.mul
        i32.const 32
        i32.add
        call $av_buffer_alloc
        tee_local $l0
        i32.store offset=304
        get_local $l0
        i32.eqz
        if $I4
          i32.const -1
          return
        end
        get_local $l2
        get_local $l0
        i32.load offset=4
        i32.store
        get_local $p0
        i32.const 1
        i32.add
        tee_local $p0
        get_local $p2
        i32.load8_u offset=4
        i32.lt_u
        br_if $L1
      end
    end
    i32.const 0)
  (func $avcodec_get_context_defaults3 (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    i32.const 0
    i32.const 976
    call $memset
    set_local $p0
    i32.const 3644
    i32.load
    set_local $l0
    get_local $p0
    i32.const 3648
    i32.load
    i32.store offset=48
    get_local $p0
    i64.const 4294967296
    i64.store offset=896
    get_local $p0
    i64.const 4294967296
    i64.store offset=888
    get_local $p0
    i64.const 4294967296
    i64.store offset=100 align=4
    get_local $p0
    get_local $l0
    i32.store offset=8
    get_local $p0
    i32.const 30
    i32.store offset=820
    get_local $p0
    i32.const 31
    i32.store offset=816
    get_local $p0
    i32.const 32
    i32.store offset=476
    get_local $p0
    i64.const 4294967296
    i64.store offset=220 align=4
    get_local $p0
    i64.const -9223372036854775808
    i64.store offset=696
    get_local $p0
    i32.const -1
    i32.store offset=416
    get_local $p0
    i32.const -1
    i32.store offset=136
    block $B0
      i32.const 3688
      i32.load
      tee_local $l0
      i32.eqz
      br_if $B0
      get_local $p0
      get_local $l0
      call $av_mallocz
      tee_local $p0
      i32.store offset=60
      get_local $p0
      br_if $B0
      i32.const -48
      return
    end
    i32.const 0)
  (func $avcodec_alloc_context3 (type $t12) (result i32)
    (local $l0 i32)
    block $B0
      i32.const 976
      call $av_malloc
      tee_local $l0
      if $I1
        get_local $l0
        call $avcodec_get_context_defaults3
        i32.const -1
        i32.gt_s
        br_if $B0
        get_local $l0
        call $free
      end
      i32.const 0
      set_local $l0
    end
    get_local $l0)
  (func $avcodec_decode_video2 (type $t9) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    get_global $g0
    i32.const 80
    i32.sub
    tee_local $l0
    set_global $g0
    get_local $l0
    get_local $p3
    i32.const 80
    call $memcpy
    set_local $l2
    i32.const -28
    set_local $l0
    block $B0
      get_local $p0
      i32.load offset=12
      tee_local $l1
      i32.eqz
      br_if $B0
      get_local $l1
      i32.load offset=8
      br_if $B0
      get_local $p2
      i32.const 0
      i32.store
      get_local $p0
      i32.load offset=124
      tee_local $l1
      get_local $p0
      i32.load offset=128
      tee_local $l3
      i32.or
      if $I1
        get_local $l1
        get_local $l3
        get_local $p0
        call $av_image_check_size
        br_if $B0
      end
      get_local $p1
      call $av_frame_unref
      block $B2
        get_local $p0
        i32.load offset=12
        tee_local $l1
        i32.load8_u offset=16
        i32.const 32
        i32.and
        br_if $B2
        get_local $p3
        i32.load offset=28
        br_if $B2
        i32.const 0
        set_local $l0
        get_local $p0
        i32.load8_u offset=808
        i32.const 1
        i32.and
        i32.eqz
        br_if $B0
      end
      get_local $p0
      get_local $p1
      get_local $p2
      get_local $l2
      get_local $l1
      i32.load offset=88
      call_indirect (type $t9)
      set_local $l0
      get_local $p2
      i32.load
      if $I3
        get_local $p0
        get_local $p0
        i32.load offset=424
        i32.const 1
        i32.add
        i32.store offset=424
        br $B0
      end
      get_local $p1
      call $av_frame_unref
    end
    get_local $l2
    i32.const 80
    i32.add
    set_global $g0
    get_local $l0)
  (func $av_image_check_size (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    block $B0 (result i32)
      get_local $p0
      i32.const 1
      i32.lt_s
      get_local $p1
      i32.const 1
      i32.lt_s
      i32.or
      i32.eqz
      if $I1
        i32.const 0
        get_local $p0
        i32.const 128
        i32.add
        i32.const 268435455
        get_local $p1
        i32.const 128
        i32.add
        i32.div_u
        i32.lt_u
        br_if $B0
        drop
      end
      i32.const -28
    end)
  (func $get_buffer_internal (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32)
    i32.const 1
    set_local $l1
    block $B0
      get_local $p0
      i32.load offset=8
      i32.eqz
      if $I1
        i32.const -28
        set_local $l0
        get_local $p0
        i32.load offset=116
        tee_local $l2
        get_local $p0
        i32.load offset=120
        tee_local $l3
        get_local $p0
        call $av_image_check_size
        i32.const 0
        i32.lt_s
        br_if $B0
        get_local $p0
        i32.load offset=136
        tee_local $l4
        i32.const 0
        i32.lt_s
        br_if $B0
        block $B2
          get_local $p1
          i32.load offset=64
          i32.const 1
          i32.ge_s
          if $I3
            get_local $p1
            i32.load offset=68
            i32.const 0
            i32.gt_s
            br_if $B2
          end
          i32.const 0
          set_local $l1
          get_local $p1
          get_local $l2
          i32.const 0
          i32.const 0
          get_local $p0
          i32.load offset=124
          i32.sub
          get_local $p0
          i32.load offset=792
          tee_local $l0
          i32.shr_s
          i32.sub
          tee_local $l5
          get_local $l2
          get_local $l5
          i32.gt_s
          select
          i32.store offset=64
          get_local $p1
          get_local $l3
          i32.const 0
          i32.const 0
          get_local $p0
          i32.load offset=128
          i32.sub
          get_local $l0
          i32.shr_s
          i32.sub
          tee_local $l0
          get_local $l0
          get_local $l3
          i32.lt_s
          select
          i32.store offset=68
        end
        get_local $p1
        get_local $l4
        i32.store offset=76
      end
      get_local $p0
      get_local $p1
      i32.const 1
      get_local $p0
      i32.load offset=476
      call_indirect (type $t3)
      set_local $l0
      get_local $p0
      i32.load offset=8
      get_local $l1
      i32.or
      br_if $B0
      get_local $p1
      get_local $p0
      i32.load offset=116
      i32.store offset=64
      get_local $p1
      get_local $p0
      i32.load offset=120
      i32.store offset=68
    end
    get_local $l0)
  (func $av_init_packet (type $t1) (param $p0 i32)
    get_local $p0
    i64.const 0
    i64.store offset=72
    get_local $p0
    i64.const -1
    i64.store offset=64
    get_local $p0
    i64.const -9223372036854775808
    i64.store offset=16
    get_local $p0
    i64.const -9223372036854775808
    i64.store offset=8
    get_local $p0
    i64.const 0
    i64.store offset=32
    get_local $p0
    i32.const 0
    i32.store
    get_local $p0
    i64.const 0
    i64.store offset=40
    get_local $p0
    i32.const 0
    i32.store offset=48)
  (func $ff_init_cabac_decoder (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    (local $l0 i32)
    get_local $p0
    get_local $p1
    i32.store offset=12
    get_local $p0
    get_local $p1
    get_local $p2
    i32.add
    i32.store offset=20
    get_local $p0
    get_local $p1
    i32.const 1
    i32.add
    i32.store offset=16
    get_local $p1
    i32.load8_u
    set_local $p2
    get_local $p0
    get_local $p1
    i32.const 2
    i32.add
    i32.store offset=16
    get_local $p0
    get_local $p2
    i32.const 18
    i32.shl
    tee_local $p2
    i32.store
    get_local $p1
    i32.load8_u offset=1
    set_local $l0
    get_local $p0
    get_local $p1
    i32.const 3
    i32.add
    i32.store offset=16
    get_local $p0
    get_local $l0
    i32.const 10
    i32.shl
    get_local $p2
    i32.or
    tee_local $p2
    i32.store
    get_local $p1
    i32.load8_u offset=2
    set_local $p1
    get_local $p0
    i32.const 510
    i32.store offset=4
    get_local $p0
    get_local $p2
    get_local $p1
    i32.const 2
    i32.shl
    i32.or
    i32.const 2
    i32.or
    i32.store)
  (func $cabac_tableinit (type $t11)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32)
    loop $L0
      get_local $l0
      i32.const 4784
      i32.add
      get_local $l0
      if $I1 (result i32)
        get_local $l0
        i32.const 65280
        i32.and
        tee_local $l1
        i32.eqz
        i32.const 3
        i32.shl
        get_local $l0
        i32.const 8
        i32.shr_u
        get_local $l0
        get_local $l1
        select
        i32.const 3264
        i32.add
        i32.load8_u
        i32.sub
      else
        i32.const 9
      end
      i32.store8
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 512
      i32.ne
      br_if $L0
    end
    i32.const 0
    set_local $l1
    loop $L2
      get_local $l1
      i32.const 1
      i32.shl
      set_local $l2
      i32.const 0
      set_local $l0
      loop $L3
        get_local $l0
        i32.const 7
        i32.shl
        get_local $l2
        i32.add
        tee_local $l3
        i32.const 1
        i32.or
        i32.const 5296
        i32.add
        get_local $l1
        i32.const 2
        i32.shl
        get_local $l0
        i32.add
        i32.const 2816
        i32.add
        i32.load8_u
        tee_local $l4
        i32.store8
        get_local $l3
        i32.const 5296
        i32.add
        get_local $l4
        i32.store8
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        i32.const 4
        i32.ne
        br_if $L3
      end
      get_local $l2
      i32.const 5936
      i32.add
      get_local $l1
      i32.const 3072
      i32.add
      i32.load8_u
      i32.const 1
      i32.shl
      tee_local $l0
      i32.store8
      get_local $l2
      i32.const 5937
      i32.add
      get_local $l0
      i32.const 1
      i32.or
      i32.store8
      i32.const 5934
      get_local $l2
      i32.sub
      block $B4 (result i32)
        get_local $l1
        if $I5
          i32.const 5935
          get_local $l2
          i32.sub
          get_local $l1
          i32.const 3136
          i32.add
          i32.load8_u
          i32.const 1
          i32.shl
          tee_local $l0
          i32.store8
          get_local $l0
          i32.const 1
          i32.or
          br $B4
        end
        i32.const 5935
        get_local $l2
        i32.sub
        i32.const 1
        i32.store8
        i32.const 0
      end
      i32.store8
      get_local $l1
      i32.const 1
      i32.add
      tee_local $l1
      i32.const 64
      i32.ne
      br_if $L2
    end
    i32.const 6064
    i32.const 3200
    i32.const 63
    call $memcpy
    drop)
  (func $get_bits (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_local $p0
    i32.load
    get_local $p0
    i32.load offset=8
    tee_local $l1
    i32.const 3
    i32.shr_u
    i32.add
    i32.load align=1
    set_local $l0
    get_local $p0
    get_local $p1
    get_local $l1
    i32.add
    tee_local $l2
    get_local $p0
    i32.load offset=16
    tee_local $p0
    get_local $p0
    get_local $l2
    i32.gt_u
    select
    i32.store offset=8
    get_local $l0
    i32.const 8
    i32.shr_u
    i32.const 65280
    i32.and
    get_local $l0
    i32.const 24
    i32.shr_u
    i32.or
    get_local $l0
    i32.const 8
    i32.shl
    i32.const 16711680
    i32.and
    get_local $l0
    i32.const 24
    i32.shl
    i32.or
    i32.or
    get_local $l1
    i32.const 7
    i32.and
    i32.shl
    i32.const 32
    get_local $p1
    i32.sub
    i32.shr_u)
  (func $show_bits (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    i32.load
    get_local $p0
    i32.load offset=8
    tee_local $l0
    i32.const 3
    i32.shr_u
    i32.add
    i32.load align=1
    tee_local $p0
    i32.const 8
    i32.shr_u
    i32.const 65280
    i32.and
    get_local $p0
    i32.const 24
    i32.shr_u
    i32.or
    get_local $p0
    i32.const 8
    i32.shl
    i32.const 16711680
    i32.and
    get_local $p0
    i32.const 24
    i32.shl
    i32.or
    i32.or
    get_local $l0
    i32.const 7
    i32.and
    i32.shl
    i32.const 24
    i32.shr_u)
  (func $skip_bits (type $t4) (param $p0 i32) (param $p1 i32)
    get_local $p0
    get_local $p0
    i32.load offset=8
    get_local $p1
    i32.add
    tee_local $p1
    get_local $p0
    i32.load offset=16
    tee_local $p0
    get_local $p0
    get_local $p1
    i32.gt_u
    select
    i32.store offset=8)
  (func $get_bits1 (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load
    get_local $p0
    i32.load offset=8
    tee_local $l0
    i32.const 3
    i32.shr_u
    i32.add
    i32.load8_u
    set_local $l1
    get_local $p0
    get_local $l0
    get_local $l0
    get_local $p0
    i32.load offset=16
    i32.lt_s
    i32.add
    i32.store offset=8
    get_local $l1
    get_local $l0
    i32.const 7
    i32.and
    i32.shl
    i32.const 7
    i32.shr_u
    i32.const 1
    i32.and)
  (func $get_bits_long (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    get_local $p1
    i32.eqz
    if $I0
      i32.const 0
      return
    end
    get_local $p1
    i32.const 25
    i32.le_s
    if $I1
      get_local $p0
      get_local $p1
      call $get_bits
      return
    end
    get_local $p0
    i32.const 16
    call $get_bits
    get_local $p1
    i32.const 16
    i32.sub
    tee_local $p1
    i32.shl
    get_local $p0
    get_local $p1
    call $get_bits
    i32.or)
  (func $get_ue_golomb_long (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.const 31
    get_local $p0
    call $show_bits_long
    tee_local $l0
    i32.const 65535
    i32.gt_u
    i32.const 4
    i32.shl
    tee_local $l1
    i32.const 8
    i32.or
    get_local $l1
    get_local $l0
    get_local $l0
    i32.const 16
    i32.shr_u
    get_local $l0
    i32.const 65536
    i32.lt_u
    select
    tee_local $l0
    i32.const 65280
    i32.and
    tee_local $l1
    select
    get_local $l0
    i32.const 8
    i32.shr_u
    get_local $l0
    get_local $l1
    select
    i32.const 3264
    i32.add
    i32.load8_u
    i32.add
    tee_local $l0
    i32.sub
    call $skip_bits_long
    get_local $p0
    i32.const 32
    get_local $l0
    i32.sub
    call $get_bits_long
    i32.const 1
    i32.sub)
  (func $show_bits_long (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_global $g0
    i32.const 32
    i32.sub
    tee_local $l0
    set_global $g0
    get_local $l0
    get_local $p0
    i32.load offset=16
    i32.store offset=24
    get_local $l0
    get_local $p0
    i64.load offset=8 align=4
    i64.store offset=16
    get_local $l0
    get_local $p0
    i64.load align=4
    i64.store offset=8
    get_local $l0
    i32.const 8
    i32.add
    i32.const 32
    call $get_bits_long
    set_local $p0
    get_local $l0
    i32.const 32
    i32.add
    set_global $g0
    get_local $p0)
  (func $skip_bits_long (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.const 0
    get_local $p0
    i32.load offset=8
    tee_local $l0
    i32.sub
    tee_local $l1
    get_local $p0
    i32.load offset=16
    get_local $l0
    i32.sub
    tee_local $p0
    get_local $p1
    get_local $p0
    get_local $p1
    i32.lt_s
    select
    get_local $p1
    get_local $l1
    i32.lt_s
    select
    get_local $l0
    i32.add
    i32.store offset=8)
  (func $get_se_golomb_long (type $t0) (param $p0 i32) (result i32)
    get_local $p0
    call $get_ue_golomb_long
    tee_local $p0
    i32.const 1
    i32.add
    i32.const 1
    i32.shr_u
    i32.const 0
    get_local $p0
    i32.const 1
    i32.shr_u
    i32.sub
    get_local $p0
    i32.const 1
    i32.and
    select)
  (func $av_malloc (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    i32.const 3736
    i32.load
    i32.const 32
    i32.sub
    set_local $l1
    loop $L0
      get_local $p0
      get_local $l1
      i32.gt_u
      if $I1
        i32.const 0
        return
      end
      get_local $p0
      call $malloc
      set_local $l0
      get_local $p0
      i32.eqz
      if $I2
        i32.const 1
        set_local $p0
        get_local $l0
        i32.eqz
        br_if $L0
      end
    end
    get_local $l0)
  (func $av_realloc (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32)
    get_local $p1
    i32.const 3736
    i32.load
    i32.const 32
    i32.sub
    i32.le_u
    if $I0 (result i32)
      get_local $p0
      get_local $p1
      i32.eqz
      get_local $p1
      i32.add
      call $realloc
    else
      get_local $l0
    end)
  (func $av_realloc_f (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l0
    set_global $g0
    block $B0
      get_local $p1
      get_local $l0
      i32.const 12
      i32.add
      call $av_size_mult
      i32.eqz
      if $I1
        get_local $p0
        get_local $l0
        i32.load offset=12
        tee_local $l1
        call $av_realloc
        tee_local $p1
        get_local $l1
        i32.eqz
        i32.or
        br_if $B0
      end
      get_local $p0
      call $free
      i32.const 0
      set_local $p1
    end
    get_local $l0
    i32.const 16
    i32.add
    set_global $g0
    get_local $p1)
  (func $av_size_mult (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i64)
    get_local $p0
    i64.extend_u/i32
    i64.const 2
    i64.shl
    set_local $l0
    block $B0 (result i32)
      get_local $p0
      i32.const 4
      i32.or
      i32.const 65536
      i32.ge_u
      if $I1
        i32.const -28
        get_local $l0
        i64.const 32
        i64.shr_u
        i32.wrap/i64
        br_if $B0
        drop
      end
      get_local $p1
      get_local $l0
      i64.store32
      i32.const 0
    end)
  (func $av_freep (type $t1) (param $p0 i32)
    get_local $p0
    i32.load
    call $free
    get_local $p0
    i32.const 0
    i32.store)
  (func $av_reallocp_array (type $t4) (param $p0 i32) (param $p1 i32)
    get_local $p0
    get_local $p0
    i32.load
    get_local $p1
    call $av_realloc_f
    i32.store)
  (func $av_mallocz (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    call $av_malloc
    tee_local $l0
    if $I0
      get_local $l0
      i32.const 0
      get_local $p0
      call $memset
      drop
    end
    get_local $l0)
  (func $ff_fast_malloc (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    get_local $p2
    get_local $p1
    i32.load
    i32.ge_u
    if $I0
      get_local $p0
      i32.load
      call $free
      get_local $p0
      get_local $p2
      i32.const 17
      i32.mul
      i32.const 4
      i32.shr_u
      i32.const 32
      i32.add
      tee_local $p0
      get_local $p2
      get_local $p0
      get_local $p2
      i32.gt_u
      select
      tee_local $p0
      call $av_malloc
      tee_local $p2
      i32.store
      get_local $p1
      get_local $p0
      i32.const 0
      get_local $p2
      select
      i32.store
    end)
  (func $av_malloc_array (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32)
    block $B0
      get_local $p1
      i32.eqz
      br_if $B0
      i32.const 2147483647
      get_local $p1
      i32.div_u
      get_local $p0
      i32.le_u
      br_if $B0
      get_local $p0
      get_local $p1
      i32.mul
      call $av_malloc
      set_local $l0
    end
    get_local $l0)
  (func $av_mallocz_array (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32)
    block $B0
      get_local $p1
      i32.eqz
      br_if $B0
      i32.const 2147483647
      get_local $p1
      i32.div_u
      get_local $p0
      i32.le_u
      br_if $B0
      get_local $p0
      get_local $p1
      i32.mul
      call $av_mallocz
      set_local $l0
    end
    get_local $l0)
  (func $av_buffer_create (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l1
    set_global $g0
    get_local $l1
    i32.const 24
    call $av_mallocz
    tee_local $l0
    i32.store offset=12
    block $B0
      get_local $l0
      i32.eqz
      br_if $B0
      get_local $l0
      i32.const 0
      i32.store offset=16
      get_local $l0
      get_local $p1
      i32.store offset=4
      get_local $l0
      get_local $p0
      i32.store
      get_local $l0
      i32.const 1
      i32.store offset=8
      get_local $l0
      get_local $p2
      i32.const 33
      get_local $p2
      select
      i32.store offset=12
      i32.const 12
      call $av_mallocz
      tee_local $p2
      i32.eqz
      if $I1
        get_local $l1
        i32.const 12
        i32.add
        call $av_freep
        br $B0
      end
      get_local $p2
      get_local $p1
      i32.store offset=8
      get_local $p2
      get_local $p0
      i32.store offset=4
      get_local $p2
      get_local $l0
      i32.store
      get_local $p2
      set_local $l2
    end
    get_local $l1
    i32.const 16
    i32.add
    set_global $g0
    get_local $l2)
  (func $av_buffer_default_free (type $t4) (param $p0 i32) (param $p1 i32)
    get_local $p1
    call $free)
  (func $av_buffer_alloc (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l0
    set_global $g0
    get_local $l0
    get_local $p0
    call $av_malloc
    tee_local $l1
    i32.store offset=12
    block $B0
      get_local $l1
      if $I1
        get_local $l1
        get_local $p0
        i32.const 33
        call $av_buffer_create
        tee_local $p0
        br_if $B0
        get_local $l0
        i32.const 12
        i32.add
        call $av_freep
      end
      i32.const 0
      set_local $p0
    end
    get_local $l0
    i32.const 16
    i32.add
    set_global $g0
    get_local $p0)
  (func $av_buffer_allocz (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    call $av_buffer_alloc
    tee_local $l0
    if $I0
      get_local $l0
      i32.load offset=4
      i32.const 0
      get_local $p0
      call $memset
      drop
    end
    get_local $l0)
  (func $av_buffer_ref (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    i32.const 12
    call $av_mallocz
    tee_local $l0
    i32.eqz
    if $I0
      i32.const 0
      return
    end
    get_local $l0
    get_local $p0
    i64.load align=4
    i64.store align=4
    get_local $l0
    get_local $p0
    i32.load offset=8
    i32.store offset=8
    get_local $p0
    i32.load
    i32.const 8
    i32.add
    i32.const 1
    call $atomic_int_add_and_fetch_gcc
    drop
    get_local $l0)
  (func $atomic_int_add_and_fetch_gcc (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    get_local $p0
    get_local $p0
    i32.load
    get_local $p1
    i32.add
    tee_local $p0
    i32.store
    get_local $p0)
  (func $av_buffer_unref (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l1
    set_global $g0
    block $B0
      get_local $p0
      i32.eqz
      br_if $B0
      get_local $p0
      i32.load
      tee_local $l0
      i32.eqz
      br_if $B0
      get_local $l1
      get_local $l0
      i32.load
      tee_local $l0
      i32.store offset=12
      get_local $p0
      call $av_freep
      get_local $l0
      i32.const 8
      i32.add
      i32.const -1
      call $atomic_int_add_and_fetch_gcc
      br_if $B0
      get_local $l0
      i32.load offset=16
      get_local $l0
      i32.load
      get_local $l0
      i32.load offset=12
      call_indirect (type $t4)
      get_local $l1
      i32.const 12
      i32.add
      call $av_freep
    end
    get_local $l1
    i32.const 16
    i32.add
    set_global $g0)
  (func $av_frame_alloc (type $t12) (result i32)
    (local $l0 i32)
    i32.const 400
    call $av_mallocz
    tee_local $l0
    i32.eqz
    if $I0
      i32.const 0
      return
    end
    get_local $l0
    call $get_frame_defaults
    get_local $l0)
  (func $get_frame_defaults (type $t1) (param $p0 i32)
    get_local $p0
    i32.const 0
    i32.const 400
    call $memset
    tee_local $p0
    i64.const -9223372036854775808
    i64.store offset=144
    get_local $p0
    i64.const -9223372036854775808
    i64.store offset=136
    get_local $p0
    i64.const -9223372036854775808
    i64.store offset=128
    get_local $p0
    i64.const -9223372036854775808
    i64.store offset=360
    get_local $p0
    i64.const 0
    i64.store offset=376
    get_local $p0
    i64.const -1
    i64.store offset=368
    get_local $p0
    i32.const -1
    i32.store offset=392
    get_local $p0
    i64.const 4294967296
    i64.store offset=120
    get_local $p0
    i64.const 8589934594
    i64.store offset=348 align=4
    get_local $p0
    i64.const 8589934591
    i64.store offset=76 align=4
    get_local $p0
    i32.const 0
    i32.store offset=356
    get_local $p0
    i64.const 8589934592
    i64.store offset=340 align=4)
  (func $av_frame_free (type $t1) (param $p0 i32)
    (local $l0 i32)
    block $B0
      get_local $p0
      i32.eqz
      br_if $B0
      get_local $p0
      i32.load
      tee_local $l0
      i32.eqz
      br_if $B0
      get_local $l0
      call $av_frame_unref
      get_local $p0
      call $av_freep
    end)
  (func $av_frame_unref (type $t1) (param $p0 i32)
    (local $l0 i32)
    loop $L0
      get_local $p0
      get_local $l0
      i32.const 2
      i32.shl
      i32.add
      i32.const 304
      i32.add
      call $av_buffer_unref
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 8
      i32.ne
      br_if $L0
    end
    get_local $p0
    call $get_frame_defaults)
  (func $av_frame_ref (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    get_local $p1
    i32.load offset=76
    i32.store offset=76
    get_local $p0
    get_local $p1
    i32.load offset=64
    i32.store offset=64
    get_local $p0
    get_local $p1
    i32.load offset=68
    i32.store offset=68
    get_local $p0
    get_local $p1
    i32.load offset=388
    i32.store offset=388
    get_local $p0
    get_local $p1
    i64.load offset=296
    i64.store offset=296
    get_local $p0
    get_local $p1
    i32.load offset=72
    i32.store offset=72
    get_local $p1
    i32.load offset=304
    tee_local $l0
    if $I0
      loop $L1
        block $B2
          get_local $l0
          i32.eqz
          br_if $B2
          get_local $p0
          get_local $l1
          i32.const 2
          i32.shl
          i32.add
          get_local $l0
          call $av_buffer_ref
          tee_local $l0
          i32.store offset=304
          get_local $l0
          br_if $B2
          get_local $p0
          call $av_frame_unref
          i32.const -48
          return
        end
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        i32.const 8
        i32.ne
        if $I3
          get_local $p1
          get_local $l1
          i32.const 2
          i32.shl
          i32.add
          i32.load offset=304
          set_local $l0
          br $L1
        end
      end
      get_local $p0
      get_local $p1
      i64.load
      i64.store
      get_local $p0
      get_local $p1
      i64.load offset=24
      i64.store offset=24
      get_local $p0
      get_local $p1
      i64.load offset=16
      i64.store offset=16
      get_local $p0
      get_local $p1
      i64.load offset=8
      i64.store offset=8
      get_local $p0
      get_local $p1
      i64.load offset=32
      i64.store offset=32
      get_local $p0
      get_local $p1
      i64.load offset=40
      i64.store offset=40
      get_local $p0
      get_local $p1
      i64.load offset=48
      i64.store offset=48
      get_local $p0
      get_local $p1
      i64.load offset=56
      i64.store offset=56
      i32.const 0
      return
    end
    call $abort
    unreachable)
  (func $av_pix_fmt_desc_get (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    loop $L0
      get_local $p0
      get_local $l0
      i32.const 24
      i32.mul
      i32.const 3520
      i32.add
      i32.load
      i32.eq
      if $I1
        get_local $l0
        i32.const 24
        i32.mul
        i32.const 3524
        i32.add
        return
      end
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 4
      i32.ne
      br_if $L0
    end
    i32.const 0)
  (func $bpg_decoder_get_data (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32)
    get_local $p2
    i32.const 3
    i32.const 1
    get_local $p0
    i32.load offset=24
    select
    tee_local $l0
    i32.lt_s
    if $I0
      get_local $p1
      get_local $p0
      i32.load offset=8
      get_local $p2
      i32.const 2
      i32.shl
      i32.add
      tee_local $p0
      i32.load offset=32
      i32.store
      get_local $p0
      i32.load
      return
    end
    block $B1
      get_local $p2
      get_local $l0
      i32.ne
      br_if $B1
      get_local $p0
      i32.load8_u offset=29
      i32.eqz
      br_if $B1
      get_local $p1
      get_local $p0
      i32.load offset=12
      tee_local $p0
      i32.load offset=32
      i32.store
      get_local $p0
      i32.load
      return
    end
    get_local $p1
    i32.const 0
    i32.store
    i32.const 0)
  (func $bpg_decoder_get_info (export "bpg_decoder_get_info") (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load offset=8
    i32.eqz
    if $I0
      i32.const -1
      return
    end
    get_local $p1
    get_local $p0
    i32.load offset=16
    i32.store
    get_local $p1
    get_local $p0
    i32.load offset=20
    i32.store offset=4
    get_local $p1
    get_local $p0
    i32.load offset=24
    i32.store8 offset=8
    get_local $p1
    get_local $p0
    i32.load8_u offset=31
    tee_local $l0
    i32.eqz
    get_local $p0
    i32.load8_u offset=29
    i32.const 0
    i32.ne
    i32.and
    i32.store8 offset=9
    get_local $p0
    i32.load8_u offset=33
    set_local $l1
    get_local $p1
    get_local $l0
    i32.store8 offset=13
    get_local $p1
    get_local $l1
    i32.store8 offset=12
    get_local $p1
    get_local $p0
    i32.load8_u offset=32
    i32.store8 offset=14
    get_local $p1
    get_local $p0
    i32.load offset=36
    i32.store8 offset=10
    get_local $p1
    get_local $p0
    i32.load8_u offset=30
    i32.store8 offset=11
    get_local $p1
    get_local $p0
    i32.load8_u offset=34
    i32.store8 offset=15
    get_local $p1
    get_local $p0
    i32.load16_u offset=48
    i32.store16 offset=16
    i32.const 0)
  (func $bpg_decoder_start (export "bpg_decoder_start") (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32)
    i32.const -1
    set_local $l0
    block $B0
      get_local $p0
      i32.load offset=8
      i32.eqz
      br_if $B0
      get_local $p0
      i32.load8_u offset=68
      br_if $B0
      get_local $p0
      get_local $p1
      call $bpg_decoder_output_init
      tee_local $l0
      br_if $B0
      get_local $p0
      get_local $p1
      i32.store offset=72
      i32.const 1
      set_local $l0
      get_local $p0
      i32.const 1
      i32.store8 offset=68
      i32.const 0
      set_local $p1
      get_local $p0
      get_local $p0
      get_local $p0
      i32.const 108
      i32.add
      i32.const 0
      call $bpg_decoder_get_data
      i32.store offset=92
      get_local $p0
      i32.load offset=24
      if $I1
        get_local $p0
        get_local $p0
        get_local $p0
        i32.const 112
        i32.add
        i32.const 1
        call $bpg_decoder_get_data
        i32.store offset=96
        get_local $p0
        get_local $p0
        get_local $p0
        i32.const 116
        i32.add
        i32.const 2
        call $bpg_decoder_get_data
        i32.store offset=100
        i32.const 3
        set_local $l0
      end
      get_local $p0
      i32.load8_u offset=29
      if $I2
        get_local $p0
        get_local $p0
        i32.const 120
        i32.add
        get_local $l0
        call $bpg_decoder_get_data
        set_local $p1
      end
      i32.const 0
      set_local $l0
      get_local $p0
      i32.const 0
      i32.store offset=80
      get_local $p0
      get_local $p1
      i32.store offset=104
    end
    get_local $l0)
  (func $bpg_decoder_output_init (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32)
    i32.const -1
    set_local $l0
    get_local $p1
    i32.const 1
    i32.le_u
    if $I0 (result i32)
      i32.const 0
      set_local $l0
      get_local $p0
      i32.const 0
      i32.store16 offset=77 align=1
      get_local $p0
      get_local $p1
      i32.const 1
      i32.eq
      i32.store8 offset=76
      block $B1
        get_local $p0
        i32.load offset=24
        i32.const 1
        i32.sub
        i32.const 1
        i32.gt_u
        br_if $B1
        get_local $p0
        get_local $p0
        i32.load offset=16
        tee_local $p1
        i32.const 1
        i32.add
        i32.const 2
        i32.div_s
        i32.store offset=84
        get_local $p0
        get_local $p0
        i32.load offset=20
        i32.const 1
        i32.add
        i32.const 2
        i32.div_s
        i32.store offset=88
        get_local $p0
        get_local $p1
        call $av_malloc
        i32.store offset=124
        get_local $p0
        get_local $p0
        i32.load offset=16
        call $av_malloc
        i32.store offset=128
        get_local $p0
        get_local $p0
        i32.load offset=84
        i32.const 1
        i32.shl
        i32.const 14
        i32.add
        call $av_malloc
        i32.store offset=196
        get_local $p0
        i32.load offset=24
        i32.const 1
        i32.ne
        br_if $B1
        loop $L2
          get_local $p0
          get_local $l0
          i32.const 2
          i32.shl
          i32.add
          tee_local $p1
          get_local $p0
          i32.load offset=84
          call $av_malloc
          i32.store offset=132
          get_local $p1
          get_local $p0
          i32.load offset=84
          call $av_malloc
          i32.store offset=164
          get_local $l0
          i32.const 1
          i32.add
          tee_local $l0
          i32.const 8
          i32.ne
          br_if $L2
        end
      end
      get_local $p0
      i32.const 200
      i32.add
      get_local $p0
      i32.load8_u offset=30
      i32.const 16
      i32.const 8
      get_local $p0
      i32.load8_u offset=77
      select
      get_local $p0
      i32.load offset=36
      get_local $p0
      i32.load8_u offset=32
      call $convert_init
      get_local $p0
      block $B3 (result i32)
        i32.const 34
        get_local $p0
        i32.load offset=24
        i32.eqz
        br_if $B3
        drop
        get_local $p0
        i32.load offset=36
        i32.const 2
        i32.shl
        i32.const 3616
        i32.add
        i32.load
      end
      i32.store offset=248
      i32.const 0
    else
      get_local $l0
    end)
  (func $convert_init (type $t8) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 f64) (local $l3 f64) (local $l4 f64) (local $l5 f64) (local $l6 f64) (local $l7 f64) (local $l8 f64)
    i32.const -1
    get_local $p2
    i32.shl
    i32.const -1
    i32.xor
    f64.convert_s/i32
    i32.const 1
    i32.const 30
    get_local $p2
    i32.sub
    tee_local $l0
    i32.shl
    f64.convert_s/i32
    f64.mul
    tee_local $l2
    i32.const -1
    get_local $p1
    i32.shl
    i32.const -1
    i32.xor
    f64.convert_s/i32
    f64.div
    tee_local $l7
    set_local $l5
    get_local $l7
    set_local $l3
    get_local $p4
    if $I0
      get_local $l2
      i32.const 224
      get_local $p1
      i32.const 8
      i32.sub
      tee_local $l1
      i32.shl
      f64.convert_s/i32
      f64.div
      set_local $l3
      get_local $l2
      i32.const 219
      get_local $l1
      i32.shl
      f64.convert_s/i32
      f64.div
      set_local $l5
    end
    f64.const 0x1.d2f1a9fbe76c9p-4 (;=0.114;)
    set_local $l4
    f64.const 0x1.322d0e5604189p-2 (;=0.299;)
    set_local $l2
    block $B1
      block $B2
        block $B3
          block $B4
            get_local $p3
            br_table $B2 $B1 $B1 $B4 $B3 $B1
          end
          f64.const 0x1.27bb2fec56d5dp-4 (;=0.0722;)
          set_local $l4
          f64.const 0x1.b367a0f9096bcp-3 (;=0.2126;)
          set_local $l2
          br $B2
        end
        f64.const 0x1.e5c91d14e3bcdp-5 (;=0.0593;)
        set_local $l4
        f64.const 0x1.0d013a92a3055p-2 (;=0.2627;)
        set_local $l2
      end
      get_local $p0
      get_local $l3
      f64.const 0x1p+0 (;=1;)
      get_local $l4
      f64.sub
      tee_local $l6
      get_local $l6
      f64.add
      f64.mul
      call $lrint
      i32.store offset=32
      get_local $p0
      get_local $l3
      f64.const 0x1p+0 (;=1;)
      get_local $l2
      f64.sub
      tee_local $l8
      get_local $l8
      f64.add
      f64.mul
      call $lrint
      i32.store offset=20
      get_local $p0
      get_local $l3
      get_local $l2
      get_local $l2
      f64.add
      get_local $l8
      f64.mul
      get_local $l6
      get_local $l2
      f64.sub
      tee_local $l2
      f64.div
      f64.mul
      call $lrint
      i32.store offset=28
      get_local $p0
      get_local $l3
      get_local $l4
      get_local $l4
      f64.add
      get_local $l6
      f64.mul
      get_local $l2
      f64.div
      f64.mul
      call $lrint
      i32.store offset=24
    end
    get_local $p0
    get_local $l0
    i32.store
    get_local $p0
    get_local $l7
    call $lrint
    tee_local $p3
    i32.store offset=8
    get_local $p0
    i32.const 1
    get_local $p1
    i32.const 1
    i32.sub
    i32.shl
    i32.store offset=36
    get_local $p0
    i32.const 1
    i32.const 29
    get_local $p2
    i32.sub
    i32.shl
    tee_local $p2
    i32.store offset=4
    get_local $p4
    if $I5
      get_local $l5
      call $lrint
      tee_local $p3
      i32.const -16
      get_local $p1
      i32.const 8
      i32.sub
      i32.shl
      i32.mul
      get_local $p2
      i32.add
      set_local $p2
    end
    get_local $p0
    get_local $p4
    i32.store offset=44
    get_local $p0
    get_local $p1
    i32.store offset=40
    get_local $p0
    get_local $p2
    i32.store offset=16
    get_local $p0
    get_local $p3
    i32.store offset=12)
  (func $gray_to_rgb24 (type $t10) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32)
    (local $l0 i32) (local $l1 i32)
    block $B0
      block $B1
        get_local $p0
        i32.load offset=40
        i32.const 8
        i32.ne
        br_if $B1
        get_local $p0
        i32.load offset=44
        br_if $B1
        get_local $p5
        i32.const 1
        i32.lt_s
        br_if $B0
        i32.const 0
        set_local $p0
        loop $L2
          get_local $p1
          get_local $p0
          get_local $p2
          i32.add
          i32.load8_u
          tee_local $p3
          i32.store8 offset=2
          get_local $p1
          get_local $p3
          i32.store8 offset=1
          get_local $p1
          get_local $p3
          i32.store8
          get_local $p1
          get_local $p6
          i32.add
          set_local $p1
          get_local $p0
          i32.const 1
          i32.add
          tee_local $p0
          get_local $p5
          i32.ne
          br_if $L2
        end
        br $B0
      end
      get_local $p5
      i32.const 1
      i32.lt_s
      br_if $B0
      get_local $p0
      i32.load
      set_local $p4
      get_local $p0
      i32.load offset=16
      set_local $l0
      get_local $p0
      i32.load offset=12
      set_local $l1
      i32.const 0
      set_local $p0
      loop $L3
        get_local $p1
        get_local $l1
        get_local $p0
        get_local $p2
        i32.add
        i32.load8_u
        i32.mul
        get_local $l0
        i32.add
        get_local $p4
        i32.shr_s
        call $clamp8
        tee_local $p3
        i32.store8 offset=2
        get_local $p1
        get_local $p3
        i32.store8 offset=1
        get_local $p1
        get_local $p3
        i32.store8
        get_local $p1
        get_local $p6
        i32.add
        set_local $p1
        get_local $p0
        i32.const 1
        i32.add
        tee_local $p0
        get_local $p5
        i32.ne
        br_if $L3
      end
    end)
  (func $bpg_decoder_get_frame_duration (export "bpg_decoder_get_frame_duration") (type $t5) (param $p0 i32) (param $p1 i32) (param $p2 i32)
    get_local $p1
    i32.const 0
    i32.store
    get_local $p2
    i32.const 1
    i32.store)
  (func $bpg_decoder_get_line (export "bpg_decoder_get_line") (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32)
    i32.const -1
    set_local $l0
    block $B0
      get_local $p0
      i32.load offset=80
      tee_local $l4
      get_local $p0
      i32.load offset=20
      i32.ge_u
      br_if $B0
      get_local $p0
      i32.load offset=108
      get_local $l4
      i32.mul
      set_local $l1
      get_local $p0
      i32.load offset=92
      set_local $l6
      i32.const 4
      set_local $l5
      get_local $p0
      i32.load8_u offset=76
      i32.eqz
      if $I1
        i32.const 4
        i32.const 3
        get_local $p0
        i32.load8_u offset=78
        select
        set_local $l5
      end
      get_local $l1
      get_local $l6
      i32.add
      set_local $l6
      get_local $p0
      i32.load offset=16
      set_local $l1
      block $B2
        block $B3
          block $B4
            block $B5
              block $B6
                get_local $p0
                i32.load offset=24
                br_table $B6 $B5 $B4 $B3 $B0
              end
              get_local $p0
              i32.const 200
              i32.add
              get_local $p1
              get_local $l6
              i32.const 0
              i32.const 0
              get_local $l1
              get_local $l5
              get_local $p0
              i32.load offset=248
              call_indirect (type $t10)
              br $B2
            end
            get_local $l4
            i32.eqz
            if $I7
              i32.const 0
              set_local $l0
              loop $L8
                i32.const 0
                set_local $l2
                get_local $l0
                i32.const 4
                i32.gt_u
                tee_local $l3
                i32.eqz
                if $I9
                  get_local $l0
                  i32.const 8
                  i32.sub
                  get_local $l0
                  get_local $l3
                  select
                  tee_local $l2
                  get_local $p0
                  i32.load offset=88
                  tee_local $l3
                  i32.const 1
                  i32.sub
                  get_local $l2
                  get_local $l3
                  i32.lt_s
                  select
                  set_local $l2
                end
                get_local $p0
                i32.load offset=100
                set_local $l3
                get_local $p0
                i32.load offset=116
                set_local $l7
                get_local $p0
                get_local $l0
                i32.const 2
                i32.shl
                i32.add
                tee_local $l8
                i32.load offset=132
                get_local $p0
                i32.load offset=96
                get_local $p0
                i32.load offset=112
                get_local $l2
                i32.mul
                i32.add
                get_local $p0
                i32.load offset=84
                call $memcpy
                drop
                get_local $l8
                i32.load offset=164
                get_local $l3
                get_local $l2
                get_local $l7
                i32.mul
                i32.add
                get_local $p0
                i32.load offset=84
                call $memcpy
                drop
                get_local $l0
                i32.const 1
                i32.add
                tee_local $l0
                i32.const 8
                i32.ne
                br_if $L8
              end
            end
            get_local $p0
            i32.load offset=124
            get_local $p0
            i32.const 132
            i32.add
            get_local $l1
            get_local $l4
            i32.const 1
            i32.shr_s
            tee_local $l3
            i32.const 8
            i32.rem_s
            tee_local $l0
            get_local $p0
            i32.load offset=196
            get_local $p0
            i32.load8_u offset=30
            get_local $l4
            i32.const 1
            i32.and
            tee_local $l2
            get_local $p0
            i32.load8_u offset=28
            call $interp2_vh
            get_local $p0
            i32.load offset=128
            get_local $p0
            i32.const 164
            i32.add
            get_local $l1
            get_local $l0
            get_local $p0
            i32.load offset=196
            get_local $p0
            i32.load8_u offset=30
            get_local $l2
            get_local $p0
            i32.load8_u offset=28
            call $interp2_vh
            get_local $l2
            if $I10
              get_local $p0
              i32.load offset=100
              set_local $l2
              get_local $p0
              i32.load offset=116
              set_local $l7
              get_local $p0
              get_local $l0
              i32.const 24
              i32.shl
              i32.const 83886080
              i32.add
              i32.const 24
              i32.shr_s
              i32.const 8
              i32.rem_s
              i32.const 24
              i32.shl
              i32.const 24
              i32.shr_s
              i32.const 2
              i32.shl
              i32.add
              tee_local $l0
              i32.load offset=132
              get_local $p0
              i32.load offset=96
              get_local $l3
              i32.const 5
              i32.add
              tee_local $l3
              get_local $p0
              i32.load offset=88
              tee_local $l8
              i32.const 1
              i32.sub
              get_local $l3
              get_local $l8
              i32.lt_s
              select
              tee_local $l3
              get_local $p0
              i32.load offset=112
              i32.mul
              i32.add
              get_local $p0
              i32.load offset=84
              call $memcpy
              drop
              get_local $l0
              i32.load offset=164
              get_local $l2
              get_local $l3
              get_local $l7
              i32.mul
              i32.add
              get_local $p0
              i32.load offset=84
              call $memcpy
              drop
            end
            get_local $p0
            i32.const 200
            i32.add
            get_local $p1
            get_local $l6
            get_local $p0
            i32.load offset=124
            get_local $p0
            i32.load offset=128
            get_local $l1
            get_local $l5
            get_local $p0
            i32.load offset=248
            call_indirect (type $t10)
            br $B2
          end
          get_local $p0
          i32.load offset=100
          set_local $l0
          get_local $p0
          i32.load offset=116
          set_local $l2
          get_local $p0
          i32.load offset=124
          get_local $p0
          i32.load offset=96
          get_local $p0
          i32.load offset=112
          get_local $l4
          i32.mul
          i32.add
          get_local $l1
          get_local $p0
          i32.load8_u offset=30
          get_local $p0
          i32.load8_u offset=28
          get_local $p0
          i32.load offset=196
          call $interp2_h
          get_local $p0
          i32.load offset=128
          get_local $l0
          get_local $l2
          get_local $l4
          i32.mul
          i32.add
          get_local $l1
          get_local $p0
          i32.load8_u offset=30
          get_local $p0
          i32.load8_u offset=28
          get_local $p0
          i32.load offset=196
          call $interp2_h
          get_local $p0
          i32.const 200
          i32.add
          get_local $p1
          get_local $l6
          get_local $p0
          i32.load offset=124
          get_local $p0
          i32.load offset=128
          get_local $l1
          get_local $l5
          get_local $p0
          i32.load offset=248
          call_indirect (type $t10)
          br $B2
        end
        get_local $p0
        i32.const 200
        i32.add
        get_local $p1
        get_local $l6
        get_local $p0
        i32.load offset=96
        get_local $p0
        i32.load offset=112
        get_local $l4
        i32.mul
        i32.add
        get_local $p0
        i32.load offset=100
        get_local $p0
        i32.load offset=116
        get_local $l4
        i32.mul
        i32.add
        get_local $l1
        get_local $l5
        get_local $p0
        i32.load offset=248
        call_indirect (type $t10)
      end
      block $B11
        get_local $p0
        i32.load8_u offset=31
        if $I12
          get_local $p0
          i32.const 200
          i32.add
          get_local $p1
          get_local $p0
          i32.load offset=104
          get_local $p0
          i32.load offset=120
          get_local $l4
          i32.mul
          i32.add
          get_local $l1
          get_local $l5
          call $alpha_combine8
          get_local $p0
          i32.load8_u offset=76
          i32.eqz
          br_if $B11
          get_local $p1
          i32.const 3
          i32.add
          get_local $l1
          call $put_dummy_gray8
          br $B11
        end
        get_local $p0
        i32.load8_u offset=76
        i32.eqz
        br_if $B11
        get_local $p0
        i32.load8_u offset=29
        if $I13
          get_local $p0
          i32.const 200
          i32.add
          get_local $p1
          i32.const 3
          i32.add
          get_local $p0
          i32.load offset=104
          get_local $p0
          i32.load offset=120
          get_local $l4
          i32.mul
          i32.add
          get_local $l1
          call $gray_to_gray8
          get_local $p0
          i32.load8_u offset=33
          i32.eqz
          br_if $B11
          get_local $p1
          get_local $l1
          call $alpha_divide8
          br $B11
        end
        get_local $p1
        i32.const 3
        i32.add
        get_local $l1
        call $put_dummy_gray8
      end
      get_local $p0
      get_local $p0
      i32.load offset=80
      i32.const 1
      i32.add
      i32.store offset=80
      i32.const 0
      set_local $l0
    end
    get_local $l0)
  (func $interp2_vh (type $t16) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32)
    get_local $p2
    i32.const 1
    i32.add
    i32.const 2
    i32.div_s
    set_local $l0
    i32.const 1
    get_local $p5
    i32.const 8
    i32.sub
    tee_local $l1
    i32.shl
    i32.const 1
    i32.shr_s
    set_local $l2
    get_local $p1
    get_local $p3
    i32.const 7
    i32.and
    i32.const 2
    i32.shl
    i32.add
    i32.load
    set_local $l3
    get_local $p1
    get_local $p3
    i32.const 3
    i32.add
    i32.const 7
    i32.and
    i32.const 2
    i32.shl
    i32.add
    i32.load
    set_local $l4
    get_local $p1
    get_local $p3
    i32.const 2
    i32.add
    i32.const 7
    i32.and
    i32.const 2
    i32.shl
    i32.add
    i32.load
    set_local $l5
    get_local $p1
    get_local $p3
    i32.const 1
    i32.add
    i32.const 7
    i32.and
    i32.const 2
    i32.shl
    i32.add
    i32.load
    set_local $l6
    get_local $p1
    get_local $p3
    i32.const 1
    i32.sub
    i32.const 7
    i32.and
    i32.const 2
    i32.shl
    i32.add
    i32.load
    set_local $l7
    get_local $p1
    get_local $p3
    i32.const 6
    i32.add
    i32.const 7
    i32.and
    i32.const 2
    i32.shl
    i32.add
    i32.load
    set_local $l8
    get_local $p1
    get_local $p3
    i32.const 5
    i32.add
    i32.const 7
    i32.and
    i32.const 2
    i32.shl
    i32.add
    i32.load
    set_local $p3
    block $B0
      get_local $p6
      if $I1
        get_local $p2
        i32.const 1
        i32.lt_s
        br_if $B0
        get_local $l0
        i32.const 1
        get_local $l0
        i32.const 1
        i32.gt_s
        select
        set_local $p6
        i32.const 0
        set_local $p1
        loop $L2
          get_local $p1
          i32.const 1
          i32.shl
          get_local $p4
          i32.add
          get_local $l2
          get_local $p1
          get_local $p3
          i32.add
          i32.load8_u
          i32.sub
          get_local $p1
          get_local $l8
          i32.add
          i32.load8_u
          i32.const 2
          i32.shl
          i32.add
          get_local $p1
          get_local $l7
          i32.add
          i32.load8_u
          i32.const -10
          i32.mul
          i32.add
          get_local $p1
          get_local $l3
          i32.add
          i32.load8_u
          i32.const 57
          i32.mul
          i32.add
          get_local $p1
          get_local $l6
          i32.add
          i32.load8_u
          i32.const 18
          i32.mul
          i32.add
          get_local $p1
          get_local $l5
          i32.add
          i32.load8_u
          i32.const -6
          i32.mul
          i32.add
          get_local $p1
          get_local $l4
          i32.add
          i32.load8_u
          i32.const 1
          i32.shl
          i32.add
          get_local $l1
          i32.shr_s
          i32.store16 offset=6
          get_local $p1
          i32.const 1
          i32.add
          tee_local $p1
          get_local $p6
          i32.ne
          br_if $L2
        end
        br $B0
      end
      get_local $p2
      i32.const 1
      i32.lt_s
      br_if $B0
      get_local $l0
      i32.const 1
      get_local $l0
      i32.const 1
      i32.gt_s
      select
      set_local $p6
      i32.const 0
      set_local $p1
      loop $L3
        get_local $p1
        i32.const 1
        i32.shl
        get_local $p4
        i32.add
        get_local $p1
        get_local $p3
        i32.add
        i32.load8_u
        i32.const 1
        i32.shl
        get_local $l2
        i32.add
        get_local $p1
        get_local $l8
        i32.add
        i32.load8_u
        i32.const -6
        i32.mul
        i32.add
        get_local $p1
        get_local $l7
        i32.add
        i32.load8_u
        i32.const 18
        i32.mul
        i32.add
        get_local $p1
        get_local $l3
        i32.add
        i32.load8_u
        i32.const 57
        i32.mul
        i32.add
        get_local $p1
        get_local $l6
        i32.add
        i32.load8_u
        i32.const -10
        i32.mul
        i32.add
        get_local $p1
        get_local $l5
        i32.add
        i32.load8_u
        i32.const 2
        i32.shl
        i32.add
        get_local $p1
        get_local $l4
        i32.add
        i32.load8_u
        i32.sub
        get_local $l1
        i32.shr_s
        i32.store16 offset=6
        get_local $p1
        i32.const 1
        i32.add
        tee_local $p1
        get_local $p6
        i32.ne
        br_if $L3
      end
    end
    get_local $p4
    i32.const 6
    i32.add
    set_local $p3
    get_local $p4
    i32.load16_u offset=6
    set_local $p6
    i32.const 0
    set_local $p1
    loop $L4
      get_local $p4
      get_local $p1
      i32.const 1
      i32.shl
      i32.add
      get_local $p6
      i32.store16
      get_local $p1
      i32.const 1
      i32.add
      tee_local $p1
      i32.const 3
      i32.ne
      br_if $L4
    end
    get_local $l0
    i32.const 3
    i32.add
    set_local $p6
    get_local $l0
    i32.const 1
    i32.shl
    get_local $p4
    i32.add
    i32.load16_u offset=4
    set_local $l0
    i32.const 0
    set_local $p1
    loop $L5
      get_local $p4
      get_local $p1
      get_local $p6
      i32.add
      i32.const 1
      i32.shl
      i32.add
      get_local $l0
      i32.store16
      get_local $p1
      i32.const 1
      i32.add
      tee_local $p1
      i32.const 4
      i32.ne
      br_if $L5
    end
    get_local $p7
    i32.eqz
    if $I6
      get_local $p0
      get_local $p3
      get_local $p2
      get_local $p5
      call $interp2p0_simple16
      return
    end
    get_local $p0
    get_local $p3
    get_local $p2
    get_local $p5
    call $interp2p1_simple16)
  (func $interp2_h (type $t6) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_local $p5
    i32.const 3
    i32.add
    get_local $p1
    get_local $p2
    i32.const 1
    i32.add
    i32.const 2
    i32.div_s
    tee_local $l0
    call $memcpy
    set_local $l1
    get_local $p5
    get_local $p1
    i32.load8_u
    tee_local $l2
    i32.store8 offset=2
    get_local $p5
    get_local $l2
    i32.const 257
    i32.mul
    i32.store16 align=1
    get_local $p5
    get_local $l0
    i32.add
    get_local $p1
    get_local $l0
    i32.add
    i32.const 1
    i32.sub
    i32.load8_u
    i32.const 16843009
    i32.mul
    i32.store offset=3 align=1
    get_local $p4
    i32.eqz
    if $I0
      get_local $p0
      get_local $l1
      get_local $p2
      get_local $p3
      call $interp2p0_simple
      return
    end
    get_local $p0
    get_local $l1
    get_local $p2
    get_local $p3
    call $interp2p1_simple)
  (func $alpha_combine8 (type $t8) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_local $p3
    i32.const 1
    i32.ge_s
    if $I0
      i32.const 1
      get_local $p0
      i32.load offset=40
      tee_local $p0
      i32.const 1
      i32.sub
      i32.shl
      set_local $l0
      loop $L1
        get_local $p1
        get_local $p2
        get_local $l1
        i32.add
        i32.load8_u
        tee_local $l2
        get_local $p1
        i32.load8_u
        i32.mul
        get_local $l0
        i32.add
        get_local $p0
        i32.shr_s
        i32.store8
        get_local $p1
        get_local $l2
        get_local $p1
        i32.load8_u offset=1
        i32.mul
        get_local $l0
        i32.add
        get_local $p0
        i32.shr_s
        i32.store8 offset=1
        get_local $p1
        get_local $l2
        get_local $p1
        i32.load8_u offset=2
        i32.mul
        get_local $l0
        i32.add
        get_local $p0
        i32.shr_s
        i32.store8 offset=2
        get_local $p1
        get_local $p4
        i32.add
        set_local $p1
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        get_local $p3
        i32.ne
        br_if $L1
      end
    end)
  (func $put_dummy_gray8 (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32)
    get_local $p1
    i32.const 1
    i32.ge_s
    if $I0
      loop $L1
        get_local $p0
        i32.const 255
        i32.store8
        get_local $p0
        i32.const 4
        i32.add
        set_local $p0
        get_local $l0
        i32.const 1
        i32.add
        tee_local $l0
        get_local $p1
        i32.ne
        br_if $L1
      end
    end)
  (func $gray_to_gray8 (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    block $B0
      get_local $p0
      i32.load offset=40
      i32.const 8
      i32.eq
      if $I1
        get_local $p3
        i32.const 1
        i32.lt_s
        br_if $B0
        i32.const 0
        set_local $p0
        loop $L2
          get_local $p1
          get_local $p0
          get_local $p2
          i32.add
          i32.load8_u
          i32.store8
          get_local $p1
          i32.const 4
          i32.add
          set_local $p1
          get_local $p0
          i32.const 1
          i32.add
          tee_local $p0
          get_local $p3
          i32.ne
          br_if $L2
        end
        br $B0
      end
      get_local $p3
      i32.const 1
      i32.lt_s
      br_if $B0
      get_local $p0
      i32.load
      set_local $l0
      get_local $p0
      i32.load offset=4
      set_local $l1
      get_local $p0
      i32.load offset=8
      set_local $l2
      i32.const 0
      set_local $p0
      loop $L3
        get_local $p1
        get_local $l2
        get_local $p0
        get_local $p2
        i32.add
        i32.load8_u
        i32.mul
        get_local $l1
        i32.add
        get_local $l0
        i32.shr_s
        i32.store8
        get_local $p1
        i32.const 4
        i32.add
        set_local $p1
        get_local $p0
        i32.const 1
        i32.add
        tee_local $p0
        get_local $p3
        i32.ne
        br_if $L3
      end
    end)
  (func $alpha_divide8 (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    i32.const 6128
    i32.load8_u
    i32.eqz
    if $I0
      i32.const 6128
      i32.const 1
      i32.store8
      call $alpha_divide8_init
    end
    get_local $p1
    i32.const 1
    i32.ge_s
    if $I1
      loop $L2
        block $B3
          get_local $p0
          i32.load8_u offset=3
          tee_local $l0
          i32.eqz
          if $I4
            get_local $p0
            i32.const 255
            i32.store8 offset=2
            get_local $p0
            i32.const 65535
            i32.store16 align=1
            br $B3
          end
          get_local $p0
          get_local $p0
          i32.load8_u
          get_local $l0
          get_local $l0
          i32.const 2
          i32.shl
          i32.const 6144
          i32.add
          i32.load
          tee_local $l2
          call $comp_divide8
          i32.store8
          get_local $p0
          get_local $p0
          i32.load8_u offset=1
          get_local $l0
          get_local $l2
          call $comp_divide8
          i32.store8 offset=1
          get_local $p0
          get_local $p0
          i32.load8_u offset=2
          get_local $l0
          get_local $l2
          call $comp_divide8
          i32.store8 offset=2
        end
        get_local $p0
        i32.const 4
        i32.add
        set_local $p0
        get_local $l1
        i32.const 1
        i32.add
        tee_local $l1
        get_local $p1
        i32.ne
        br_if $L2
      end
    end)
  (func $interp2p0_simple16 (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32)
    i32.const -1
    get_local $p3
    i32.shl
    i32.const -1
    i32.xor
    set_local $l0
    i32.const 1
    i32.const 14
    get_local $p3
    i32.sub
    tee_local $l1
    i32.shl
    i32.const 1
    i32.shr_s
    set_local $l2
    block $B0
      get_local $p2
      i32.const 2
      i32.lt_s
      if $I1
        get_local $p2
        set_local $p3
        br $B0
      end
      i32.const 20
      get_local $p3
      i32.sub
      set_local $l3
      i32.const 1
      i32.const 19
      get_local $p3
      i32.sub
      i32.shl
      set_local $l4
      loop $L2
        get_local $p0
        get_local $l2
        get_local $p1
        i32.load16_s
        i32.add
        get_local $l1
        i32.shr_s
        get_local $l0
        call $clamp_pix
        i32.store8
        get_local $p0
        get_local $l4
        get_local $p1
        i32.const 6
        i32.sub
        i32.load16_s
        i32.sub
        get_local $p1
        i32.load16_s offset=8
        i32.sub
        get_local $p1
        i32.load16_s offset=6
        get_local $p1
        i32.const 4
        i32.sub
        i32.load16_s
        i32.add
        i32.const 2
        i32.shl
        i32.add
        get_local $p1
        i32.load16_s offset=4
        get_local $p1
        i32.const 2
        i32.sub
        i32.load16_s
        i32.add
        i32.const -11
        i32.mul
        i32.add
        get_local $p1
        i32.load16_s offset=2
        get_local $p1
        i32.load16_s
        i32.add
        i32.const 40
        i32.mul
        i32.add
        get_local $l3
        i32.shr_s
        get_local $l0
        call $clamp_pix
        i32.store8 offset=1
        get_local $p0
        i32.const 2
        i32.add
        set_local $p0
        get_local $p1
        i32.const 2
        i32.add
        set_local $p1
        get_local $p2
        i32.const 3
        i32.gt_s
        set_local $l5
        get_local $p2
        i32.const 2
        i32.sub
        tee_local $p3
        set_local $p2
        get_local $l5
        br_if $L2
      end
    end
    get_local $p3
    if $I3
      get_local $p0
      get_local $l2
      get_local $p1
      i32.load16_s
      i32.add
      get_local $l1
      i32.shr_s
      get_local $l0
      call $clamp_pix
      i32.store8
    end)
  (func $interp2p1_simple16 (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32)
    i32.const 20
    get_local $p3
    i32.sub
    set_local $l6
    i32.const 1
    i32.const 19
    get_local $p3
    i32.sub
    i32.shl
    set_local $l7
    i32.const -1
    get_local $p3
    i32.shl
    i32.const -1
    i32.xor
    set_local $l8
    get_local $p1
    i32.const 2
    i32.sub
    i32.load16_s
    set_local $l9
    get_local $p1
    i32.const 4
    i32.sub
    i32.load16_s
    set_local $p3
    get_local $p1
    i32.const 6
    i32.sub
    i32.load16_s
    set_local $l0
    get_local $p1
    i32.load16_s offset=4
    set_local $l1
    get_local $p1
    i32.load16_s offset=2
    set_local $l2
    get_local $p1
    i32.load16_s
    set_local $l3
    block $B0
      get_local $p2
      i32.const 2
      i32.lt_s
      if $I1
        get_local $p2
        set_local $l4
        get_local $l0
        set_local $l5
        br $B0
      end
      loop $L2
        get_local $p0
        get_local $l0
        i32.const 1
        i32.shl
        get_local $l7
        i32.add
        get_local $p3
        tee_local $l5
        i32.const -6
        i32.mul
        i32.add
        get_local $l9
        tee_local $p3
        i32.const 18
        i32.mul
        i32.add
        get_local $l3
        tee_local $l9
        i32.const 57
        i32.mul
        tee_local $l4
        i32.add
        get_local $l2
        tee_local $l3
        i32.const -10
        i32.mul
        i32.add
        get_local $l1
        tee_local $l2
        i32.const 2
        i32.shl
        i32.add
        get_local $p1
        i32.load16_s offset=6
        tee_local $l1
        i32.sub
        get_local $l6
        i32.shr_s
        get_local $l8
        call $clamp_pix
        i32.store8
        get_local $p0
        get_local $l7
        get_local $l0
        i32.sub
        get_local $l5
        i32.const 2
        i32.shl
        i32.add
        get_local $p3
        i32.const -10
        i32.mul
        i32.add
        get_local $l4
        i32.add
        get_local $l3
        i32.const 18
        i32.mul
        i32.add
        get_local $l2
        i32.const -6
        i32.mul
        i32.add
        get_local $l1
        i32.const 1
        i32.shl
        i32.add
        get_local $l6
        i32.shr_s
        get_local $l8
        call $clamp_pix
        i32.store8 offset=1
        get_local $p1
        i32.const 2
        i32.add
        set_local $p1
        get_local $p0
        i32.const 2
        i32.add
        set_local $p0
        get_local $p2
        i32.const 3
        i32.gt_s
        set_local $l10
        get_local $l5
        set_local $l0
        get_local $p2
        i32.const 2
        i32.sub
        tee_local $l4
        set_local $p2
        get_local $l10
        br_if $L2
      end
    end
    get_local $l4
    if $I3
      get_local $p0
      get_local $l5
      i32.const 1
      i32.shl
      get_local $l7
      i32.add
      get_local $p3
      i32.const -6
      i32.mul
      i32.add
      get_local $l9
      i32.const 18
      i32.mul
      i32.add
      get_local $l3
      i32.const 57
      i32.mul
      i32.add
      get_local $l2
      i32.const -10
      i32.mul
      i32.add
      get_local $l1
      i32.const 2
      i32.shl
      i32.add
      get_local $p1
      i32.load16_s offset=6
      i32.sub
      get_local $l6
      i32.shr_s
      get_local $l8
      call $clamp_pix
      i32.store8
    end)
  (func $interp2p0_simple (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32)
    block $B0
      get_local $p2
      i32.const 2
      i32.lt_s
      if $I1
        get_local $p2
        set_local $p3
        br $B0
      end
      i32.const -1
      get_local $p3
      i32.shl
      i32.const -1
      i32.xor
      set_local $l0
      loop $L2
        get_local $p0
        get_local $p1
        i32.load8_u
        i32.store8
        get_local $p0
        get_local $p1
        i32.load8_u offset=3
        get_local $p1
        i32.const 2
        i32.sub
        i32.load8_u
        i32.add
        i32.const 2
        i32.shl
        get_local $p1
        i32.const 3
        i32.sub
        i32.load8_u
        get_local $p1
        i32.load8_u offset=4
        i32.add
        i32.sub
        get_local $p1
        i32.load8_u offset=2
        get_local $p1
        i32.const 1
        i32.sub
        i32.load8_u
        i32.add
        i32.const -11
        i32.mul
        i32.add
        get_local $p1
        i32.load8_u offset=1
        get_local $p1
        i32.load8_u
        i32.add
        i32.const 40
        i32.mul
        i32.add
        i32.const 32
        i32.add
        i32.const 6
        i32.shr_s
        get_local $l0
        call $clamp_pix
        i32.store8 offset=1
        get_local $p0
        i32.const 2
        i32.add
        set_local $p0
        get_local $p1
        i32.const 1
        i32.add
        set_local $p1
        get_local $p2
        i32.const 3
        i32.gt_s
        set_local $l1
        get_local $p2
        i32.const 2
        i32.sub
        tee_local $p3
        set_local $p2
        get_local $l1
        br_if $L2
      end
    end
    get_local $p3
    if $I3
      get_local $p0
      get_local $p1
      i32.load8_u
      i32.store8
    end)
  (func $interp2p1_simple (type $t7) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32)
    i32.const -1
    get_local $p3
    i32.shl
    i32.const -1
    i32.xor
    set_local $l6
    get_local $p1
    i32.const 1
    i32.sub
    i32.load8_u
    set_local $l7
    get_local $p1
    i32.const 2
    i32.sub
    i32.load8_u
    set_local $p3
    get_local $p1
    i32.const 3
    i32.sub
    i32.load8_u
    set_local $l0
    get_local $p1
    i32.load8_u offset=2
    set_local $l1
    get_local $p1
    i32.load8_u offset=1
    set_local $l2
    get_local $p1
    i32.load8_u
    set_local $l3
    block $B0
      get_local $p2
      i32.const 2
      i32.lt_s
      if $I1
        get_local $p2
        set_local $l4
        get_local $l0
        set_local $l5
        br $B0
      end
      loop $L2
        get_local $p0
        get_local $p3
        tee_local $l5
        i32.const -6
        i32.mul
        get_local $l0
        i32.const 1
        i32.shl
        i32.add
        get_local $l7
        tee_local $p3
        i32.const 18
        i32.mul
        i32.add
        get_local $l3
        tee_local $l7
        i32.const 57
        i32.mul
        tee_local $l4
        i32.add
        get_local $l2
        tee_local $l3
        i32.const -10
        i32.mul
        i32.add
        get_local $l1
        tee_local $l2
        i32.const 2
        i32.shl
        i32.add
        get_local $p1
        i32.load8_u offset=3
        tee_local $l1
        i32.sub
        i32.const 32
        i32.add
        i32.const 6
        i32.shr_s
        get_local $l6
        call $clamp_pix
        i32.store8
        get_local $p0
        get_local $l5
        i32.const 2
        i32.shl
        get_local $l0
        i32.sub
        get_local $p3
        i32.const -10
        i32.mul
        i32.add
        get_local $l4
        i32.add
        get_local $l3
        i32.const 18
        i32.mul
        i32.add
        get_local $l2
        i32.const -6
        i32.mul
        i32.add
        get_local $l1
        i32.const 1
        i32.shl
        i32.add
        i32.const 32
        i32.add
        i32.const 6
        i32.shr_s
        get_local $l6
        call $clamp_pix
        i32.store8 offset=1
        get_local $p1
        i32.const 1
        i32.add
        set_local $p1
        get_local $p0
        i32.const 2
        i32.add
        set_local $p0
        get_local $p2
        i32.const 3
        i32.gt_s
        set_local $l8
        get_local $l5
        set_local $l0
        get_local $p2
        i32.const 2
        i32.sub
        tee_local $l4
        set_local $p2
        get_local $l8
        br_if $L2
      end
    end
    get_local $l4
    if $I3
      get_local $p0
      get_local $p3
      i32.const -6
      i32.mul
      get_local $l5
      i32.const 1
      i32.shl
      i32.add
      get_local $l7
      i32.const 18
      i32.mul
      i32.add
      get_local $l3
      i32.const 57
      i32.mul
      i32.add
      get_local $l2
      i32.const -10
      i32.mul
      i32.add
      get_local $l1
      i32.const 2
      i32.shl
      i32.add
      get_local $p1
      i32.load8_u offset=3
      i32.sub
      i32.const 32
      i32.add
      i32.const 6
      i32.shr_s
      get_local $l6
      call $clamp_pix
      i32.store8
    end)
  (func $alpha_divide8_init (type $t11)
    (local $l0 i32)
    i32.const 1
    set_local $l0
    loop $L0
      get_local $l0
      i32.const 2
      i32.shl
      i32.const 6144
      i32.add
      get_local $l0
      i32.const 1
      i32.shr_u
      i32.const 16711808
      i32.add
      get_local $l0
      i32.div_u
      i32.store
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 256
      i32.ne
      br_if $L0
    end)
  (func $comp_divide8 (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    get_local $p0
    get_local $p2
    i32.mul
    i32.const 32768
    i32.add
    i32.const 16
    i32.shr_u
    i32.const 255
    get_local $p0
    get_local $p1
    i32.lt_u
    select)
  (func $bpg_decoder_open (export "bpg_decoder_open") (type $t12) (result i32)
    i32.const 252
    call $av_mallocz)
  (func $bpg_decoder_decode (export "bpg_decoder_decode") (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32)
    get_global $g0
    i32.const 48
    i32.sub
    tee_local $l0
    set_global $g0
    block $B0
      get_local $l0
      i32.const 8
      i32.add
      get_local $p1
      get_local $p2
      get_local $p0
      i32.load8_u offset=40
      call $bpg_decode_header
      tee_local $l1
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $l0
      i32.load8_u offset=21
      set_local $l5
      get_local $l0
      i32.load offset=32
      set_local $l8
      get_local $l0
      i32.load8_u offset=20
      set_local $l6
      get_local $l0
      i32.load offset=8
      set_local $l7
      get_local $p0
      get_local $l0
      i32.load offset=12
      tee_local $l9
      i32.store offset=20
      get_local $p0
      get_local $l7
      i32.store offset=16
      i32.const 2
      set_local $l2
      block $B1
        block $B2
          block $B3
            get_local $l0
            i32.load offset=16
            tee_local $l3
            i32.const 4
            i32.sub
            br_table $B3 $B1 $B2
          end
          i32.const 1
          set_local $l2
          br $B1
        end
        i32.const 1
        set_local $l4
        get_local $l3
        set_local $l2
      end
      get_local $p0
      get_local $l6
      i32.store8 offset=29
      get_local $p0
      get_local $l4
      i32.store8 offset=28
      get_local $p0
      get_local $l2
      i32.store offset=24
      get_local $p0
      get_local $l0
      i32.load8_u offset=23
      i32.store8 offset=33
      get_local $p0
      get_local $l0
      i32.load8_u offset=22
      i32.store8 offset=31
      get_local $l0
      i32.load8_u offset=24
      set_local $l3
      get_local $p0
      get_local $l8
      i32.store offset=36
      get_local $p0
      get_local $l3
      i32.store8 offset=32
      get_local $p0
      get_local $l5
      i32.store8 offset=30
      get_local $p0
      get_local $l0
      i32.load8_u offset=25
      i32.store8 offset=34
      get_local $p0
      get_local $l0
      i32.load16_u offset=26
      i32.store16 offset=48
      get_local $p0
      get_local $l0
      i32.load16_u offset=28
      i32.store16 offset=50
      get_local $p0
      get_local $l0
      i32.load16_u offset=30
      i32.store16 offset=52
      get_local $p0
      get_local $l0
      i32.load offset=40
      i32.store offset=44
      block $B4
        get_local $l0
        i32.load offset=36
        get_local $l1
        i32.add
        get_local $p2
        i32.gt_u
        br_if $B4
        get_local $p0
        get_local $p1
        get_local $l1
        i32.add
        get_local $p2
        get_local $l1
        i32.sub
        get_local $l7
        get_local $l9
        get_local $l2
        get_local $l5
        get_local $l6
        call $hevc_decode_start
        i32.const 0
        i32.lt_s
        br_if $B4
        get_local $p0
        call $hevc_decode_end
        get_local $p0
        i32.load offset=8
        tee_local $p1
        i32.load offset=64
        get_local $p0
        i32.load offset=16
        i32.lt_s
        br_if $B4
        get_local $p1
        i32.load offset=68
        get_local $p0
        i32.load offset=20
        i32.lt_s
        br_if $B4
        get_local $p0
        i32.const -1
        i32.store offset=80
        i32.const 0
        set_local $l1
        br $B0
      end
      get_local $p0
      i32.const 8
      i32.add
      call $av_frame_free
      get_local $p0
      i32.const 12
      i32.add
      call $av_frame_free
      get_local $p0
      i32.const 0
      i32.store offset=44
      i32.const -1
      set_local $l1
    end
    get_local $l0
    i32.const 48
    i32.add
    set_global $g0
    get_local $l1)
  (func $bpg_decode_header (type $t9) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32)
    get_global $g0
    i32.const 32
    i32.sub
    tee_local $l3
    set_global $g0
    i32.const -1
    set_local $l6
    block $B0
      get_local $p2
      i32.const 6
      i32.lt_s
      br_if $B0
      get_local $p1
      i32.load8_u
      i32.const 66
      i32.ne
      br_if $B0
      get_local $p1
      i32.load8_u offset=1
      i32.const 80
      i32.ne
      br_if $B0
      get_local $p1
      i32.load8_u offset=2
      i32.const 71
      i32.ne
      br_if $B0
      get_local $p1
      i32.load8_u offset=3
      i32.const 251
      i32.ne
      br_if $B0
      get_local $p0
      get_local $p1
      i32.load8_u offset=4
      tee_local $l0
      i32.const 5
      i32.shr_u
      tee_local $l4
      i32.store offset=8
      get_local $l0
      i32.const 191
      i32.gt_u
      br_if $B0
      get_local $p0
      get_local $l0
      i32.const 15
      i32.and
      tee_local $l2
      i32.const 8
      i32.add
      i32.store8 offset=13
      get_local $l2
      i32.const 6
      i32.gt_u
      br_if $B0
      get_local $p1
      i32.load8_u offset=5
      set_local $l2
      get_local $p0
      i32.const 0
      i32.store16 offset=22
      get_local $p0
      i32.const 0
      i32.store offset=18 align=2
      get_local $p0
      i32.const 0
      i32.store16 offset=14
      get_local $p0
      i32.const 0
      i32.store8 offset=12
      get_local $p0
      get_local $l2
      i32.const 1
      i32.and
      i32.store8 offset=17
      get_local $p0
      get_local $l2
      i32.const 4
      i32.shr_u
      tee_local $l7
      i32.store offset=24
      get_local $p0
      get_local $l2
      i32.const 1
      i32.shr_u
      i32.const 1
      i32.and
      i32.store8 offset=16
      get_local $l2
      i32.const 2
      i32.shr_u
      i32.const 1
      i32.and
      set_local $l5
      block $B1
        get_local $l0
        i32.const 16
        i32.and
        if $I2
          get_local $p0
          get_local $l5
          i32.store8 offset=15
          get_local $p0
          i32.const 1
          i32.store8 offset=12
          br $B1
        end
        get_local $l5
        i32.eqz
        br_if $B1
        i32.const 1
        set_local $l1
        get_local $p0
        i32.const 1
        i32.store8 offset=14
        get_local $p0
        i32.const 1
        i32.store8 offset=12
      end
      get_local $l2
      i32.const 79
      i32.gt_u
      br_if $B0
      get_local $l4
      i32.eqz
      tee_local $l0
      get_local $l7
      i32.const 0
      i32.ne
      i32.and
      get_local $l0
      get_local $l1
      i32.and
      i32.or
      br_if $B0
      get_local $p0
      get_local $p1
      i32.const 6
      i32.add
      get_local $p2
      i32.const 6
      i32.sub
      call $get_ue
      tee_local $l0
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $p0
      i32.const 4
      i32.add
      get_local $p1
      get_local $l0
      i32.const 6
      i32.add
      tee_local $l0
      i32.add
      get_local $p2
      get_local $l0
      i32.sub
      call $get_ue
      tee_local $l1
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $p0
      i32.load
      i32.eqz
      br_if $B0
      get_local $p0
      i32.load offset=4
      i32.eqz
      br_if $B0
      get_local $p0
      i32.const 28
      i32.add
      get_local $p1
      get_local $l0
      get_local $l1
      i32.add
      tee_local $l0
      i32.add
      get_local $p2
      get_local $l0
      i32.sub
      call $get_ue
      tee_local $l1
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $l3
      i32.const 0
      i32.store offset=28
      get_local $l0
      get_local $l1
      i32.add
      set_local $l0
      block $B3
        block $B4
          get_local $l2
          i32.const 8
          i32.and
          i32.eqz
          if $I5
            get_local $p0
            i32.const 0
            i32.store offset=32
            br $B4
          end
          get_local $l3
          i32.const 28
          i32.add
          get_local $p1
          get_local $l0
          i32.add
          get_local $p2
          get_local $l0
          i32.sub
          call $get_ue
          tee_local $l2
          i32.const 0
          i32.lt_s
          br_if $B0
          get_local $p0
          i32.const 0
          i32.store offset=32
          get_local $l0
          get_local $l2
          i32.add
          tee_local $l0
          get_local $l3
          i32.load offset=28
          i32.add
          tee_local $l2
          get_local $p2
          i32.gt_s
          br_if $B0
          block $B6
            get_local $p3
            br_if $B6
            get_local $p0
            i32.load8_u offset=17
            br_if $B6
            get_local $l2
            set_local $l0
            br $B3
          end
          get_local $l0
          get_local $l2
          i32.ge_s
          br_if $B4
          get_local $p0
          i32.const 32
          i32.add
          set_local $l4
          loop $L7
            get_local $l3
            i32.const 24
            i32.add
            get_local $p1
            get_local $l0
            i32.add
            get_local $l2
            get_local $l0
            i32.sub
            call $get_ue32
            tee_local $l1
            i32.const 0
            i32.lt_s
            br_if $B0
            get_local $l3
            i32.const 20
            i32.add
            get_local $p1
            get_local $l0
            get_local $l1
            i32.add
            tee_local $l0
            i32.add
            get_local $l2
            get_local $l0
            i32.sub
            call $get_ue
            tee_local $l1
            i32.const 0
            i32.lt_s
            br_if $B0
            get_local $l3
            i32.load offset=20
            tee_local $l7
            get_local $l0
            get_local $l1
            i32.add
            tee_local $l5
            i32.add
            tee_local $l0
            get_local $l2
            i32.gt_u
            br_if $B0
            get_local $p0
            i32.load8_u offset=17
            i32.eqz
            get_local $l3
            i32.load offset=24
            tee_local $l9
            i32.const 5
            i32.ne
            i32.or
            i32.eqz
            if $I8
              get_local $l3
              i32.const 16
              i32.add
              get_local $p1
              get_local $l5
              i32.add
              get_local $l2
              get_local $l5
              i32.sub
              call $get_ue
              tee_local $l1
              i32.const 0
              i32.lt_s
              br_if $B0
              get_local $l3
              i32.const 12
              i32.add
              get_local $p1
              get_local $l1
              get_local $l5
              i32.add
              tee_local $l1
              i32.add
              get_local $l2
              get_local $l1
              i32.sub
              call $get_ue
              tee_local $l8
              i32.const 0
              i32.lt_s
              br_if $B0
              get_local $l3
              i32.const 8
              i32.add
              get_local $p1
              get_local $l1
              get_local $l8
              i32.add
              tee_local $l1
              i32.add
              get_local $l2
              get_local $l1
              i32.sub
              call $get_ue
              i32.const 0
              i32.lt_s
              br_if $B0
              get_local $l3
              i32.load offset=12
              tee_local $l1
              i32.const 1
              i32.sub
              i32.const 65534
              i32.gt_u
              br_if $B0
              get_local $l3
              i32.load offset=8
              tee_local $l8
              i32.const 1
              i32.sub
              i32.const 65534
              i32.gt_u
              br_if $B0
              get_local $l3
              i32.load offset=16
              tee_local $l10
              i32.const 65535
              i32.gt_u
              br_if $B0
              get_local $p0
              get_local $l8
              i32.store16 offset=22
              get_local $p0
              get_local $l1
              i32.store16 offset=20
              get_local $p0
              get_local $l10
              i32.store16 offset=18
            end
            get_local $p3
            if $I9
              i32.const 16
              call $av_malloc
              tee_local $l1
              i32.const 0
              i32.store offset=12
              get_local $l1
              get_local $l7
              i32.store offset=4
              get_local $l1
              get_local $l9
              i32.store
              get_local $l4
              get_local $l1
              i32.store
              get_local $l1
              get_local $l7
              call $av_malloc
              tee_local $l4
              i32.store offset=8
              get_local $l4
              get_local $p1
              get_local $l5
              i32.add
              get_local $l1
              i32.load offset=4
              call $memcpy
              drop
              get_local $l1
              i32.const 12
              i32.add
              set_local $l4
            end
            get_local $l0
            get_local $l2
            i32.lt_s
            br_if $L7
          end
        end
        get_local $p0
        i32.load8_u offset=17
        i32.eqz
        br_if $B3
        get_local $p0
        i32.load16_u offset=20
        i32.eqz
        br_if $B0
      end
      get_local $p0
      i32.load offset=28
      i32.eqz
      if $I10
        get_local $p0
        get_local $p2
        get_local $l0
        i32.sub
        i32.store offset=28
      end
      get_local $l0
      set_local $l6
    end
    get_local $l3
    i32.const 32
    i32.add
    set_global $g0
    get_local $l6)
  (func $hevc_decode_start (type $t19) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_global $g0
    i32.const 32
    i32.sub
    tee_local $l0
    set_global $g0
    get_local $l0
    i32.const 16
    i32.add
    call $dyn_buf_init
    get_local $l0
    call $dyn_buf_init
    block $B0
      get_local $p7
      if $I1 (result i32)
        get_local $l0
        i32.const 16
        i32.add
        get_local $p0
        i32.const 12
        i32.add
        get_local $p0
        i32.const 4
        i32.add
        get_local $p1
        get_local $p2
        get_local $p3
        get_local $p4
        i32.const 0
        get_local $p6
        call $hevc_decode_init1
        tee_local $p7
        i32.const 0
        i32.lt_s
        if $I2
          i32.const -1
          set_local $p7
          br $B0
        end
        get_local $p1
        get_local $p7
        i32.add
        set_local $p1
        get_local $p2
        get_local $p7
        i32.sub
      else
        get_local $p2
      end
      set_local $l1
      i32.const -1
      set_local $p7
      get_local $l0
      get_local $p0
      i32.const 8
      i32.add
      get_local $p0
      get_local $p1
      get_local $l1
      get_local $p3
      get_local $p4
      get_local $p5
      get_local $p6
      call $hevc_decode_init1
      tee_local $p3
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $p0
      get_local $l0
      i32.const 16
      i32.add
      get_local $l0
      get_local $p1
      get_local $p3
      i32.add
      get_local $l1
      get_local $p3
      i32.sub
      tee_local $p1
      call $hevc_decode_frame_internal
      set_local $p0
      get_local $l0
      i32.load offset=16
      call $free
      get_local $l0
      i32.load
      call $free
      get_local $p0
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $p2
      get_local $p1
      i32.sub
      get_local $p0
      i32.add
      set_local $p7
    end
    get_local $l0
    i32.const 32
    i32.add
    set_global $g0
    get_local $p7)
  (func $hevc_decode_end (type $t1) (param $p0 i32)
    (local $l0 i32)
    get_local $p0
    i32.load offset=4
    tee_local $l0
    if $I0
      get_local $l0
      call $avcodec_close
      get_local $p0
      i32.load offset=4
      call $free
      get_local $p0
      i32.const 0
      i32.store offset=4
    end
    get_local $p0
    i32.load
    tee_local $l0
    if $I1
      get_local $l0
      call $avcodec_close
      get_local $p0
      i32.load
      call $free
      get_local $p0
      i32.const 0
      i32.store
    end)
  (func $get_ue (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    get_local $p0
    get_local $p1
    get_local $p2
    call $get_ue32
    tee_local $p1
    i32.const 0
    i32.ge_s
    if $I0
      i32.const -1
      get_local $p1
      get_local $p0
      i32.load
      i32.const 1073741823
      i32.gt_u
      select
      return
    end
    get_local $p1)
  (func $get_ue32 (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    i32.const -1
    set_local $l2
    block $B0
      get_local $p2
      i32.const 1
      i32.lt_s
      br_if $B0
      get_local $p1
      i32.load8_u
      tee_local $l0
      i32.const 24
      i32.shl
      i32.const 24
      i32.shr_s
      tee_local $l1
      i32.const 0
      i32.ge_s
      if $I1
        get_local $p0
        get_local $l0
        i32.store
        i32.const 1
        return
      end
      get_local $l1
      i32.const -128
      i32.eq
      br_if $B0
      get_local $p1
      i32.const 1
      i32.add
      set_local $l1
      get_local $l0
      i32.const 127
      i32.and
      set_local $l0
      loop $L2
        get_local $p2
        i32.const 2
        i32.lt_s
        br_if $B0
        get_local $l1
        i32.load8_u
        tee_local $l3
        i32.const 127
        i32.and
        get_local $l0
        i32.const 7
        i32.shl
        i32.or
        set_local $l0
        get_local $l1
        i32.const 1
        i32.add
        set_local $l1
        get_local $p2
        i32.const 1
        i32.sub
        set_local $p2
        get_local $l3
        i32.const 128
        i32.and
        br_if $L2
      end
      get_local $p0
      get_local $l0
      i32.store
      get_local $l1
      get_local $p1
      i32.sub
      set_local $l2
    end
    get_local $l2)
  (func $dyn_buf_init (type $t1) (param $p0 i32)
    get_local $p0
    i32.const 0
    i32.store offset=8
    get_local $p0
    i64.const 0
    i64.store align=4)
  (func $hevc_decode_init1 (type $t21) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (param $p8 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l0
    set_global $g0
    i32.const -1
    set_local $l1
    block $B0
      get_local $l0
      i32.const 8
      i32.add
      get_local $l0
      i32.const 4
      i32.add
      get_local $p3
      get_local $p4
      get_local $p5
      get_local $p6
      get_local $p7
      get_local $p8
      call $build_msps
      tee_local $p3
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $p0
      get_local $l0
      i32.load offset=8
      tee_local $p0
      get_local $l0
      i32.load offset=4
      call $dyn_buf_push
      set_local $p4
      get_local $p0
      call $free
      get_local $p4
      i32.const 0
      i32.lt_s
      br_if $B0
      call $avcodec_alloc_context3
      tee_local $p0
      i32.eqz
      br_if $B0
      get_local $l0
      call $av_frame_alloc
      tee_local $p4
      i32.store offset=12
      get_local $p4
      i32.eqz
      br_if $B0
      get_local $p0
      get_local $p0
      i32.load offset=688
      i32.const 1
      i32.or
      i32.store offset=688
      get_local $p0
      call $avcodec_open2
      i32.const -1
      i32.le_s
      if $I1
        get_local $l0
        i32.const 12
        i32.add
        call $av_frame_free
        br $B0
      end
      get_local $p2
      get_local $p0
      i32.store
      get_local $p1
      get_local $p4
      i32.store
      get_local $p3
      set_local $l1
    end
    get_local $l0
    i32.const 16
    i32.add
    set_global $g0
    get_local $l1)
  (func $hevc_decode_frame_internal (type $t13) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32) (local $l11 i32) (local $l12 i32) (local $l13 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l4
    set_global $g0
    get_local $p0
    i32.load offset=4
    set_local $l5
    get_local $l4
    i64.const 0
    i64.store offset=8 align=4
    block $B0
      block $B1
        get_local $p4
        i32.const 1
        i32.lt_s
        if $I2
          get_local $p4
          set_local $l3
          br $B1
        end
        get_local $l4
        i32.const 8
        i32.add
        get_local $l5
        i32.const 0
        i32.ne
        tee_local $l11
        i32.const 2
        i32.shl
        i32.add
        set_local $l9
        i32.const -1
        set_local $l6
        get_local $p4
        set_local $l3
        loop $L3
          get_local $l3
          i32.const 2
          i32.const 5
          get_local $l1
          select
          i32.lt_s
          br_if $B0
          i32.const 0
          set_local $l2
          get_local $l1
          if $I4
            i32.const 3
            i32.const 4
            get_local $p3
            i32.load8_u offset=2
            select
            set_local $l2
          end
          get_local $l3
          get_local $l2
          i32.const 3
          i32.add
          i32.lt_s
          br_if $B0
          get_local $p3
          get_local $l2
          i32.add
          tee_local $l10
          i32.load8_u
          tee_local $l0
          i32.const 5
          i32.shl
          i32.const 32
          i32.and
          get_local $l10
          i32.load8_u offset=1
          i32.const 3
          i32.shr_u
          i32.or
          set_local $l8
          block $B5
            block $B6
              get_local $l0
              i32.const 120
              i32.and
              i32.const 64
              i32.eq
              get_local $l0
              i32.const 1
              i32.shr_u
              i32.const 63
              i32.and
              tee_local $l0
              i32.const 40
              i32.gt_u
              i32.or
              i32.eqz
              i32.const 0
              get_local $l0
              i32.const 39
              i32.ne
              select
              i32.eqz
              if $I7
                get_local $l7
                i32.eqz
                if $I8
                  i32.const 0
                  set_local $l7
                  br $B6
                end
                get_local $l9
                i32.load
                i32.eqz
                br_if $B6
                br $B5
              end
              get_local $l0
              i32.const 10
              i32.ge_u
              i32.const 0
              get_local $l0
              i32.const 16
              i32.sub
              i32.const 5
              i32.gt_u
              select
              br_if $B6
              get_local $l2
              i32.const 2
              i32.add
              tee_local $l0
              get_local $l3
              i32.ge_s
              br_if $B6
              get_local $p3
              get_local $l0
              i32.add
              i32.load8_s
              i32.const -1
              i32.gt_s
              br_if $B6
              get_local $l7
              if $I9
                get_local $l9
                i32.load
                br_if $B5
              end
              get_local $l5
              i32.eqz
              get_local $l8
              i32.const 1
              i32.ne
              i32.or
              i32.eqz
              if $I10
                get_local $l4
                i32.const 1
                i32.store offset=12
                br $B6
              end
              i32.const 1
              set_local $l7
              get_local $l4
              i32.const 1
              i32.store offset=8
            end
            get_local $p3
            get_local $l3
            get_local $l1
            call $find_nal_end
            tee_local $l0
            i32.const 0
            i32.lt_s
            br_if $B0
            get_local $p1
            get_local $p2
            get_local $l11
            get_local $l8
            i32.const 1
            i32.eq
            i32.and
            tee_local $l8
            select
            tee_local $l1
            get_local $l0
            get_local $l2
            i32.sub
            tee_local $l12
            i32.const 3
            i32.add
            tee_local $l13
            get_local $l1
            i32.load offset=8
            i32.add
            call $dyn_buf_resize
            i32.const 0
            i32.lt_s
            br_if $B0
            get_local $l1
            i32.load
            get_local $l1
            i32.load offset=8
            i32.add
            tee_local $l2
            i32.const 1
            i32.store8 offset=2
            get_local $l2
            i32.const 0
            i32.store16 align=1
            get_local $l2
            i32.const 3
            i32.add
            get_local $l10
            get_local $l12
            call $memcpy
            drop
            get_local $l8
            if $I11
              get_local $l2
              get_local $l2
              i32.load8_u offset=4
              i32.const 7
              i32.and
              i32.store8 offset=4
            end
            get_local $l1
            get_local $l1
            i32.load offset=8
            get_local $l13
            i32.add
            i32.store offset=8
            get_local $p3
            get_local $l0
            i32.add
            set_local $p3
            i32.const 1
            set_local $l1
            get_local $l3
            get_local $l0
            i32.sub
            tee_local $l3
            i32.const 0
            i32.gt_s
            br_if $L3
          end
        end
        get_local $p0
        i32.load offset=4
        set_local $l5
      end
      get_local $l5
      if $I12
        i32.const -1
        set_local $l6
        get_local $p1
        get_local $p1
        i32.load offset=8
        i32.const 32
        i32.add
        call $dyn_buf_resize
        i32.const 0
        i32.lt_s
        br_if $B0
        get_local $p0
        i32.load offset=4
        get_local $p0
        i32.load offset=12
        get_local $p1
        i32.load
        get_local $p1
        i32.load offset=8
        call $hevc_write_frame
        i32.const 0
        i32.lt_s
        br_if $B0
      end
      get_local $p2
      get_local $p2
      i32.load offset=8
      i32.const 32
      i32.add
      call $dyn_buf_resize
      i32.const 0
      i32.lt_s
      if $I13
        i32.const -1
        set_local $l6
        br $B0
      end
      get_local $p4
      get_local $l3
      i32.sub
      i32.const -1
      get_local $p0
      i32.load
      get_local $p0
      i32.load offset=8
      get_local $p2
      i32.load
      get_local $p2
      i32.load offset=8
      call $hevc_write_frame
      i32.const -1
      i32.gt_s
      select
      set_local $l6
    end
    get_local $l4
    i32.const 16
    i32.add
    set_global $g0
    get_local $l6)
  (func $bpg_decoder_close (export "bpg_decoder_close") (type $t1) (param $p0 i32)
    get_local $p0
    call $bpg_decoder_output_end
    get_local $p0
    i32.load offset=56
    call $free
    get_local $p0
    call $hevc_decode_end
    get_local $p0
    i32.const 8
    i32.add
    call $av_frame_free
    get_local $p0
    i32.const 12
    i32.add
    call $av_frame_free
    get_local $p0
    call $free)
  (func $bpg_decoder_output_end (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32)
    get_local $p0
    i32.load offset=124
    call $free
    get_local $p0
    i32.load offset=128
    call $free
    loop $L0
      get_local $p0
      get_local $l0
      i32.const 2
      i32.shl
      i32.add
      tee_local $l1
      i32.load offset=132
      call $free
      get_local $l1
      i32.load offset=164
      call $free
      get_local $l0
      i32.const 1
      i32.add
      tee_local $l0
      i32.const 8
      i32.ne
      br_if $L0
    end
    get_local $p0
    i32.load offset=196
    call $free)
  (func $clamp8 (type $t0) (param $p0 i32) (result i32)
    get_local $p0
    i32.const 0
    i32.ge_s
    if $I0
      get_local $p0
      i32.const 255
      get_local $p0
      i32.const 255
      i32.lt_s
      select
      return
    end
    i32.const 0)
  (func $ycc_to_rgb24 (type $t10) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32) (local $l8 i32) (local $l9 i32) (local $l10 i32)
    get_local $p5
    i32.const 1
    i32.ge_s
    if $I0
      get_local $p0
      i32.load offset=36
      set_local $l2
      get_local $p0
      i32.load
      set_local $l0
      get_local $p0
      i32.load offset=16
      set_local $l4
      get_local $p0
      i32.load offset=12
      set_local $l5
      get_local $p0
      i32.load offset=32
      set_local $l6
      get_local $p0
      i32.load offset=28
      set_local $l7
      get_local $p0
      i32.load offset=24
      set_local $l8
      get_local $p0
      i32.load offset=20
      set_local $l9
      i32.const 0
      set_local $p0
      loop $L1
        get_local $p0
        get_local $p3
        i32.add
        i32.load8_u
        set_local $l1
        get_local $p1
        get_local $l5
        get_local $p0
        get_local $p2
        i32.add
        i32.load8_u
        i32.mul
        get_local $l4
        i32.add
        tee_local $l3
        get_local $p0
        get_local $p4
        i32.add
        i32.load8_u
        get_local $l2
        i32.sub
        tee_local $l10
        get_local $l9
        i32.mul
        i32.add
        get_local $l0
        i32.shr_s
        call $clamp8
        i32.store8
        get_local $p1
        get_local $l3
        get_local $l7
        get_local $l10
        i32.mul
        get_local $l1
        get_local $l2
        i32.sub
        tee_local $l1
        get_local $l8
        i32.mul
        i32.add
        i32.sub
        get_local $l0
        i32.shr_s
        call $clamp8
        i32.store8 offset=1
        get_local $p1
        get_local $l3
        get_local $l1
        get_local $l6
        i32.mul
        i32.add
        get_local $l0
        i32.shr_s
        call $clamp8
        i32.store8 offset=2
        get_local $p1
        get_local $p6
        i32.add
        set_local $p1
        get_local $p0
        i32.const 1
        i32.add
        tee_local $p0
        get_local $p5
        i32.ne
        br_if $L1
      end
    end)
  (func $rgb_to_rgb24 (type $t10) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    block $B0
      block $B1
        get_local $p0
        i32.load offset=40
        i32.const 8
        i32.ne
        br_if $B1
        get_local $p0
        i32.load offset=44
        br_if $B1
        get_local $p5
        i32.const 1
        i32.lt_s
        br_if $B0
        i32.const 0
        set_local $p0
        loop $L2
          get_local $p1
          get_local $p0
          get_local $p4
          i32.add
          i32.load8_u
          i32.store8
          get_local $p1
          get_local $p0
          get_local $p2
          i32.add
          i32.load8_u
          i32.store8 offset=1
          get_local $p1
          get_local $p0
          get_local $p3
          i32.add
          i32.load8_u
          i32.store8 offset=2
          get_local $p1
          get_local $p6
          i32.add
          set_local $p1
          get_local $p0
          i32.const 1
          i32.add
          tee_local $p0
          get_local $p5
          i32.ne
          br_if $L2
        end
        br $B0
      end
      get_local $p5
      i32.const 1
      i32.lt_s
      br_if $B0
      get_local $p0
      i32.load
      set_local $l0
      get_local $p0
      i32.load offset=16
      set_local $l1
      get_local $p0
      i32.load offset=12
      set_local $l2
      i32.const 0
      set_local $p0
      loop $L3
        get_local $p1
        get_local $l2
        get_local $p0
        get_local $p4
        i32.add
        i32.load8_u
        i32.mul
        get_local $l1
        i32.add
        get_local $l0
        i32.shr_s
        call $clamp8
        i32.store8
        get_local $p1
        get_local $l2
        get_local $p0
        get_local $p2
        i32.add
        i32.load8_u
        i32.mul
        get_local $l1
        i32.add
        get_local $l0
        i32.shr_s
        call $clamp8
        i32.store8 offset=1
        get_local $p1
        get_local $l2
        get_local $p0
        get_local $p3
        i32.add
        i32.load8_u
        i32.mul
        get_local $l1
        i32.add
        get_local $l0
        i32.shr_s
        call $clamp8
        i32.store8 offset=2
        get_local $p1
        get_local $p6
        i32.add
        set_local $p1
        get_local $p0
        i32.const 1
        i32.add
        tee_local $p0
        get_local $p5
        i32.ne
        br_if $L3
      end
    end)
  (func $ycgco_to_rgb24 (type $t10) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32) (local $l6 i32) (local $l7 i32)
    get_local $p5
    i32.const 1
    i32.ge_s
    if $I0
      get_local $p0
      i32.load offset=36
      set_local $l3
      get_local $p0
      i32.load
      set_local $l0
      get_local $p0
      i32.load offset=16
      set_local $l1
      get_local $p0
      i32.load offset=12
      set_local $l2
      i32.const 0
      set_local $p0
      loop $L1
        get_local $p1
        get_local $p0
        get_local $p2
        i32.add
        i32.load8_u
        tee_local $l4
        get_local $p0
        get_local $p3
        i32.add
        i32.load8_u
        get_local $l3
        i32.sub
        tee_local $l5
        i32.sub
        tee_local $l6
        get_local $p0
        get_local $p4
        i32.add
        i32.load8_u
        get_local $l3
        i32.sub
        tee_local $l7
        i32.add
        get_local $l2
        i32.mul
        get_local $l1
        i32.add
        get_local $l0
        i32.shr_s
        call $clamp8
        i32.store8
        get_local $p1
        get_local $l4
        get_local $l5
        i32.add
        get_local $l2
        i32.mul
        get_local $l1
        i32.add
        get_local $l0
        i32.shr_s
        call $clamp8
        i32.store8 offset=1
        get_local $p1
        get_local $l6
        get_local $l7
        i32.sub
        get_local $l2
        i32.mul
        get_local $l1
        i32.add
        get_local $l0
        i32.shr_s
        call $clamp8
        i32.store8 offset=2
        get_local $p1
        get_local $p6
        i32.add
        set_local $p1
        get_local $p0
        i32.const 1
        i32.add
        tee_local $p0
        get_local $p5
        i32.ne
        br_if $L1
      end
    end)
  (func $clamp_pix (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    get_local $p0
    i32.const 0
    i32.ge_s
    if $I0
      get_local $p1
      get_local $p0
      get_local $p0
      get_local $p1
      i32.gt_s
      select
      return
    end
    i32.const 0)
  (func $build_msps (type $t19) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (param $p4 i32) (param $p5 i32) (param $p6 i32) (param $p7 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32) (local $l5 i32)
    get_global $g0
    i32.const 16
    i32.sub
    tee_local $l3
    set_global $g0
    get_local $p0
    i32.const 0
    i32.store
    i32.const -1
    set_local $l0
    block $B0
      get_local $l3
      i32.const 12
      i32.add
      get_local $p2
      get_local $p3
      call $get_ue
      tee_local $l4
      i32.const 0
      i32.lt_s
      br_if $B0
      get_local $l3
      i32.load offset=12
      tee_local $l5
      get_local $p3
      get_local $l4
      i32.sub
      i32.gt_u
      br_if $B0
      get_local $l5
      i32.const 10
      i32.add
      tee_local $l2
      call $av_malloc
      tee_local $l1
      get_local $p7
      i32.const 8
      i32.sub
      i32.store8 offset=9
      get_local $l1
      get_local $p5
      i32.store8 offset=8
      get_local $l1
      get_local $p5
      i32.const 8
      i32.shr_u
      i32.store8 offset=7
      get_local $l1
      get_local $p5
      i32.const 16
      i32.shr_u
      i32.store8 offset=6
      get_local $l1
      get_local $p5
      i32.const 24
      i32.shr_u
      i32.store8 offset=5
      get_local $l1
      get_local $p4
      i32.store8 offset=4
      get_local $l1
      get_local $p4
      i32.const 8
      i32.shr_u
      i32.store8 offset=3
      get_local $l1
      get_local $p4
      i32.const 16
      i32.shr_u
      i32.store8 offset=2
      get_local $l1
      get_local $p4
      i32.const 24
      i32.shr_u
      i32.store8 offset=1
      get_local $l1
      get_local $p6
      i32.store8
      get_local $l1
      i32.const 10
      i32.add
      get_local $p2
      get_local $l4
      i32.add
      get_local $l5
      call $memcpy
      drop
      i32.const 6
      set_local $l0
      get_local $l2
      i32.const 1
      i32.shl
      i32.const 6
      i32.add
      call $av_malloc
      tee_local $p5
      i32.const 352
      i32.store16 offset=4 align=1
      get_local $p5
      i32.const 16777216
      i32.store align=1
      block $B1
        block $B2
          block $B3
            get_local $l2
            i32.const 1
            i32.lt_s
            br_if $B3
            i32.const 0
            set_local $p2
            loop $L4
              block $B5
                block $B6
                  get_local $p2
                  get_local $l1
                  i32.add
                  i32.load8_u
                  tee_local $p4
                  get_local $p2
                  i32.const 1
                  i32.add
                  tee_local $p3
                  get_local $l2
                  i32.ge_s
                  i32.or
                  br_if $B6
                  i32.const 0
                  set_local $p4
                  get_local $p3
                  get_local $l1
                  i32.add
                  i32.load8_u
                  br_if $B6
                  get_local $p5
                  get_local $l0
                  i32.add
                  i32.const 0
                  i32.store16 align=1
                  get_local $p5
                  get_local $l0
                  i32.const 2
                  i32.add
                  tee_local $l0
                  i32.add
                  i32.const 3
                  i32.store8
                  get_local $p2
                  i32.const 2
                  i32.add
                  set_local $p3
                  br $B5
                end
                get_local $p5
                get_local $l0
                i32.add
                get_local $p4
                i32.store8
              end
              get_local $l0
              i32.const 1
              i32.add
              set_local $l0
              get_local $l2
              get_local $p3
              tee_local $p2
              i32.gt_s
              br_if $L4
            end
            get_local $l0
            br_if $B3
            i32.const 0
            set_local $l0
            br $B2
          end
          get_local $p5
          get_local $l0
          i32.add
          i32.const 1
          i32.sub
          i32.load8_u
          br_if $B1
        end
        get_local $p5
        get_local $l0
        i32.add
        i32.const 128
        i32.store8
        get_local $l0
        i32.const 1
        i32.add
        set_local $l0
      end
      get_local $l1
      call $free
      get_local $p1
      get_local $l0
      i32.store
      get_local $p0
      get_local $p5
      i32.store
      get_local $l4
      get_local $l5
      i32.add
      set_local $l0
    end
    get_local $l3
    i32.const 16
    i32.add
    set_global $g0
    get_local $l0)
  (func $dyn_buf_push (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    get_local $p0
    get_local $p0
    i32.load offset=8
    get_local $p2
    i32.add
    call $dyn_buf_resize
    i32.const 0
    i32.lt_s
    if $I0
      i32.const -1
      return
    end
    get_local $p0
    i32.load
    get_local $p0
    i32.load offset=8
    i32.add
    get_local $p1
    get_local $p2
    call $memcpy
    drop
    get_local $p0
    get_local $p0
    i32.load offset=8
    get_local $p2
    i32.add
    i32.store offset=8
    i32.const 0)
  (func $find_nal_end (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32)
    block $B0
      block $B1
        get_local $p2
        i32.eqz
        if $I2
          br $B1
        end
        block $B3
          get_local $p1
          i32.const 4
          i32.ge_s
          if $I4
            i32.const -1
            set_local $p2
            get_local $p0
            i32.load8_u
            br_if $B0
            get_local $p0
            i32.load8_u offset=1
            br_if $B3
            get_local $p0
            i32.load8_u offset=2
            br_if $B3
            i32.const 4
            set_local $l0
            get_local $p0
            i32.load8_u offset=3
            i32.const 1
            i32.ne
            br_if $B3
            br $B1
          end
          i32.const -1
          set_local $p2
          get_local $p1
          i32.const 3
          i32.ne
          br_if $B0
          get_local $p0
          i32.load8_u
          br_if $B0
        end
        get_local $p0
        i32.load8_u offset=1
        br_if $B0
        i32.const 3
        set_local $l0
        get_local $p0
        i32.load8_u offset=2
        i32.const 1
        i32.ne
        br_if $B0
      end
      i32.const -1
      set_local $p2
      get_local $l0
      i32.const 2
      i32.add
      tee_local $l1
      get_local $p1
      i32.gt_s
      br_if $B0
      get_local $p1
      get_local $l1
      i32.gt_s
      if $I5
        get_local $p1
        i32.const 2
        i32.sub
        set_local $l2
        loop $L6
          get_local $l0
          tee_local $p2
          i32.const 1
          i32.add
          set_local $l0
          block $B7
            block $B8
              get_local $p0
              get_local $p2
              i32.add
              i32.load8_u
              br_if $B8
              get_local $p0
              get_local $l0
              i32.add
              i32.load8_u
              br_if $B8
              get_local $p0
              get_local $l1
              i32.add
              tee_local $l3
              i32.load8_u
              i32.const 1
              i32.eq
              br_if $B0
              get_local $p2
              i32.const 3
              i32.add
              tee_local $l1
              get_local $p1
              i32.ge_s
              br_if $B7
              get_local $l3
              i32.load8_u
              br_if $B7
              get_local $p0
              get_local $l1
              i32.add
              i32.load8_u
              i32.const 1
              i32.eq
              br_if $B0
              br $B7
            end
            get_local $p2
            i32.const 3
            i32.add
            set_local $l1
          end
          get_local $l0
          get_local $l2
          i32.ne
          br_if $L6
        end
      end
      get_local $p1
      set_local $p2
    end
    get_local $p2)
  (func $dyn_buf_resize (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32)
    get_local $p1
    get_local $p0
    i32.load offset=4
    tee_local $l0
    i32.gt_s
    if $I0
      get_local $p0
      i32.load
      get_local $p1
      get_local $l0
      i32.const 3
      i32.mul
      i32.const 2
      i32.div_s
      tee_local $l0
      get_local $p1
      get_local $l0
      i32.gt_s
      select
      tee_local $p1
      call $av_realloc
      tee_local $l0
      i32.eqz
      if $I1
        i32.const -1
        return
      end
      get_local $p0
      get_local $p1
      i32.store offset=4
      get_local $p0
      get_local $l0
      i32.store
    end
    i32.const 0)
  (func $hevc_write_frame (type $t9) (param $p0 i32) (param $p1 i32) (param $p2 i32) (param $p3 i32) (result i32)
    (local $l0 i32)
    get_global $g0
    i32.const 96
    i32.sub
    tee_local $l0
    set_global $g0
    get_local $l0
    i32.const 16
    i32.add
    call $av_init_packet
    get_local $l0
    get_local $p2
    i32.store offset=40
    get_local $l0
    get_local $p3
    i32.store offset=44
    get_local $p2
    get_local $p3
    i32.add
    tee_local $p2
    i64.const 0
    i64.store align=1
    get_local $p2
    i64.const 0
    i64.store offset=24 align=1
    get_local $p2
    i64.const 0
    i64.store offset=16 align=1
    get_local $p2
    i64.const 0
    i64.store offset=8 align=1
    get_local $p0
    get_local $p1
    get_local $l0
    i32.const 12
    i32.add
    get_local $l0
    i32.const 16
    i32.add
    call $avcodec_decode_video2
    set_local $p0
    get_local $l0
    i32.load offset=12
    set_local $p1
    get_local $l0
    i32.const 96
    i32.add
    set_global $g0
    i32.const 0
    get_local $p1
    i32.eqz
    get_local $p0
    i32.const 0
    i32.lt_s
    i32.or
    i32.sub)
  (func $malloc (export "malloc") (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32) (local $l3 i32) (local $l4 i32)
    block $B0
      get_local $p0
      i32.const 1
      i32.sub
      i32.const 2147483582
      i32.gt_u
      br_if $B0
      i32.const 7172
      i32.load
      tee_local $l0
      i32.eqz
      if $I1
        i32.const 64
        call $sbrk
        tee_local $l0
        i32.const -1
        i32.eq
        br_if $B0
        i32.const 0
        call $sbrk
        set_local $l1
        i32.const 7172
        i32.const 7168
        i32.store
        i32.const 7176
        get_local $l1
        i32.store
        i32.const 7168
        i32.const 7168
        i32.store
        i32.const 7184
        i32.const 7180
        i32.store
        i32.const 7180
        i32.const 7180
        i32.store
        get_local $l0
        i32.const 16
        i32.add
        tee_local $l1
        i32.const 170
        call $set_block_state
        get_local $l1
        i32.const 7180
        call $list_add
        get_local $l0
        i32.const 24
        i32.add
        i32.const 7168
        call $list_add
        i32.const 7172
        i32.load
        set_local $l0
      end
      block $B2
        block $B3 (result i32)
          block $B4
            get_local $p0
            i32.const 40
            i32.add
            i32.const -32
            i32.and
            tee_local $l2
            get_local $l0
            i32.const 8
            i32.sub
            tee_local $l3
            call $get_block_size
            tee_local $l4
            i32.lt_u
            br_if $B4
            i32.const 0
            set_local $l1
            i32.const 7168
            i32.load
            tee_local $p0
            get_local $l0
            i32.ne
            if $I5
              loop $L6
                get_local $l2
                get_local $l4
                i32.eq
                if $I7
                  get_local $l0
                  set_local $l1
                  br $B2
                end
                get_local $l2
                get_local $l0
                i32.load offset=4
                tee_local $l0
                i32.const 8
                i32.sub
                tee_local $l3
                call $get_block_size
                tee_local $l4
                i32.lt_u
                br_if $B4
                get_local $p0
                get_local $l0
                i32.ne
                br_if $L6
              end
            end
            get_local $l2
            get_local $l4
            i32.sub
            i32.const 32
            i32.add
            call $sbrk
            i32.const -1
            i32.eq
            br_if $B0
            i32.const 7176
            i32.const 0
            call $sbrk
            i32.store
            get_local $p0
            br $B3
          end
          get_local $l0
        end
        set_local $l1
        get_local $l2
        get_local $l3
        i32.add
        tee_local $p0
        get_local $l3
        call $list_add
        get_local $p0
        i32.const 8
        i32.add
        get_local $l1
        call $list_add
        get_local $p0
        i32.const 170
        call $set_block_state
      end
      get_local $l1
      call $list_del
      get_local $l3
      i32.const 85
      call $set_block_state
    end
    get_local $l1)
  (func $set_block_state (type $t4) (param $p0 i32) (param $p1 i32)
    get_local $p0
    i32.const 1
    i32.sub
    get_local $p1
    i32.store8)
  (func $list_add (type $t4) (param $p0 i32) (param $p1 i32)
    (local $l0 i32)
    get_local $p1
    i32.load offset=4
    set_local $l0
    get_local $p1
    get_local $p0
    i32.store offset=4
    get_local $p0
    get_local $l0
    i32.store offset=4
    get_local $p0
    get_local $p1
    i32.store
    get_local $l0
    get_local $p0
    i32.store)
  (func $get_block_size (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32)
    i32.const 7176
    i32.load
    get_local $p0
    i32.load offset=4
    tee_local $l0
    get_local $l0
    i32.const 7180
    i32.eq
    select
    get_local $p0
    i32.sub)
  (func $list_del (type $t1) (param $p0 i32)
    (local $l0 i32)
    get_local $p0
    i32.load
    tee_local $l0
    get_local $p0
    i32.load offset=4
    tee_local $p0
    i32.store offset=4
    get_local $p0
    get_local $l0
    i32.store)
  (func $free (export "free") (type $t1) (param $p0 i32)
    (local $l0 i32) (local $l1 i32)
    block $B0
      get_local $p0
      i32.eqz
      br_if $B0
      get_local $p0
      i32.const 7168
      call $list_add
      get_local $p0
      i32.const 8
      i32.sub
      tee_local $l1
      i32.const 170
      call $set_block_state
      block $B1
        get_local $l1
        i32.load
        tee_local $l0
        i32.const 7180
        i32.eq
        br_if $B1
        get_local $l0
        call $get_block_state
        i32.const 170
        i32.ne
        br_if $B1
        get_local $l1
        call $list_del
        get_local $p0
        call $list_del
        get_local $l0
        set_local $l1
      end
      get_local $l1
      i32.load offset=4
      tee_local $l0
      i32.const 7180
      i32.eq
      br_if $B0
      get_local $l0
      call $get_block_state
      i32.const 170
      i32.ne
      br_if $B0
      get_local $l0
      call $list_del
      get_local $l1
      i32.const 8
      i32.add
      tee_local $p0
      call $list_del
      get_local $p0
      get_local $l0
      i32.const 8
      i32.add
      tee_local $p0
      call $list_add
      get_local $p0
      call $list_del
    end)
  (func $get_block_state (type $t0) (param $p0 i32) (result i32)
    get_local $p0
    i32.const 1
    i32.sub
    i32.load8_u)
  (func $realloc (type $t2) (param $p0 i32) (param $p1 i32) (result i32)
    (local $l0 i32)
    get_local $p0
    i32.eqz
    if $I0
      get_local $p1
      call $malloc
      return
    end
    block $B1
      block $B2
        get_local $p1
        i32.eqz
        if $I3
          get_local $p0
          call $free
          br $B2
        end
        get_local $p1
        call $malloc
        tee_local $l0
        br_if $B1
      end
      i32.const 0
      return
    end
    get_local $l0
    get_local $p0
    get_local $p1
    get_local $p0
    i32.const 4
    i32.sub
    i32.load
    get_local $p0
    i32.const -1
    i32.xor
    i32.add
    tee_local $l0
    get_local $p1
    get_local $l0
    i32.lt_u
    select
    call $memcpy
    set_local $p1
    get_local $p0
    call $free
    get_local $p1)
  (func $memcmp (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    block $B0
      get_local $p2
      i32.eqz
      br_if $B0
      loop $L1
        get_local $p0
        i32.load8_u
        tee_local $l1
        get_local $p1
        i32.load8_u
        tee_local $l2
        i32.eq
        if $I2
          get_local $p1
          i32.const 1
          i32.add
          set_local $p1
          get_local $p0
          i32.const 1
          i32.add
          set_local $p0
          get_local $p2
          i32.const 1
          i32.sub
          tee_local $p2
          br_if $L1
          br $B0
        end
      end
      get_local $l1
      get_local $l2
      i32.sub
      set_local $l0
    end
    get_local $l0)
  (func $lrint (type $t20) (param $p0 f64) (result i32)
    get_local $p0
    f64.nearest
    tee_local $p0
    f64.abs
    f64.const 0x1p+31 (;=2.14748e+09;)
    f64.lt
    if $I0
      get_local $p0
      i32.trunc_s/f64
      return
    end
    i32.const -2147483648)
  (func $__errno_location (export "__errno_location") (type $t12) (result i32)
    i32.const 7188)
  (func $sbrk (type $t0) (param $p0 i32) (result i32)
    (local $l0 i32) (local $l1 i32)
    i32.const 3740
    i32.load
    tee_local $l0
    get_local $p0
    i32.const 3
    i32.add
    i32.const -4
    i32.and
    tee_local $l1
    i32.add
    set_local $p0
    block $B0
      get_local $l1
      i32.const 1
      i32.ge_s
      i32.const 0
      get_local $p0
      get_local $l0
      i32.le_u
      select
      br_if $B0
      current_memory
      i32.const 16
      i32.shl
      get_local $p0
      i32.lt_u
      if $I1
        get_local $p0
        call $emscripten_resize_heap
        i32.eqz
        br_if $B0
      end
      i32.const 3740
      get_local $p0
      i32.store
      get_local $l0
      return
    end
    i32.const 7188
    i32.const 48
    i32.store
    i32.const -1)
  (func $memcpy (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i32)
    get_local $p2
    i32.const 512
    i32.ge_u
    if $I0
      get_local $p0
      get_local $p1
      get_local $p2
      call $emscripten_memcpy_big
      drop
      get_local $p0
      return
    end
    get_local $p0
    get_local $p2
    i32.add
    set_local $l0
    block $B1
      get_local $p0
      get_local $p1
      i32.xor
      i32.const 3
      i32.and
      i32.eqz
      if $I2
        block $B3
          get_local $p2
          i32.const 1
          i32.lt_s
          if $I4
            get_local $p0
            set_local $p2
            br $B3
          end
          get_local $p0
          i32.const 3
          i32.and
          i32.eqz
          if $I5
            get_local $p0
            set_local $p2
            br $B3
          end
          get_local $p0
          set_local $p2
          loop $L6
            get_local $p2
            get_local $p1
            i32.load8_u
            i32.store8
            get_local $p1
            i32.const 1
            i32.add
            set_local $p1
            get_local $p2
            i32.const 1
            i32.add
            tee_local $p2
            get_local $l0
            i32.ge_u
            br_if $B3
            get_local $p2
            i32.const 3
            i32.and
            br_if $L6
          end
        end
        block $B7
          get_local $l0
          i32.const -4
          i32.and
          tee_local $l1
          i32.const 64
          i32.lt_u
          br_if $B7
          get_local $p2
          get_local $l1
          i32.const -64
          i32.add
          tee_local $l2
          i32.gt_u
          br_if $B7
          loop $L8
            get_local $p2
            get_local $p1
            i32.load
            i32.store
            get_local $p2
            get_local $p1
            i32.load offset=4
            i32.store offset=4
            get_local $p2
            get_local $p1
            i32.load offset=8
            i32.store offset=8
            get_local $p2
            get_local $p1
            i32.load offset=12
            i32.store offset=12
            get_local $p2
            get_local $p1
            i32.load offset=16
            i32.store offset=16
            get_local $p2
            get_local $p1
            i32.load offset=20
            i32.store offset=20
            get_local $p2
            get_local $p1
            i32.load offset=24
            i32.store offset=24
            get_local $p2
            get_local $p1
            i32.load offset=28
            i32.store offset=28
            get_local $p2
            get_local $p1
            i32.load offset=32
            i32.store offset=32
            get_local $p2
            get_local $p1
            i32.load offset=36
            i32.store offset=36
            get_local $p2
            get_local $p1
            i32.load offset=40
            i32.store offset=40
            get_local $p2
            get_local $p1
            i32.load offset=44
            i32.store offset=44
            get_local $p2
            get_local $p1
            i32.load offset=48
            i32.store offset=48
            get_local $p2
            get_local $p1
            i32.load offset=52
            i32.store offset=52
            get_local $p2
            get_local $p1
            i32.load offset=56
            i32.store offset=56
            get_local $p2
            get_local $p1
            i32.load offset=60
            i32.store offset=60
            get_local $p1
            i32.const -64
            i32.sub
            set_local $p1
            get_local $p2
            i32.const -64
            i32.sub
            tee_local $p2
            get_local $l2
            i32.le_u
            br_if $L8
          end
        end
        get_local $p2
        get_local $l1
        i32.ge_u
        br_if $B1
        loop $L9
          get_local $p2
          get_local $p1
          i32.load
          i32.store
          get_local $p1
          i32.const 4
          i32.add
          set_local $p1
          get_local $p2
          i32.const 4
          i32.add
          tee_local $p2
          get_local $l1
          i32.lt_u
          br_if $L9
        end
        br $B1
      end
      get_local $l0
      i32.const 4
      i32.lt_u
      if $I10
        get_local $p0
        set_local $p2
        br $B1
      end
      get_local $p0
      get_local $l0
      i32.const 4
      i32.sub
      tee_local $l1
      i32.gt_u
      if $I11
        get_local $p0
        set_local $p2
        br $B1
      end
      get_local $p0
      set_local $p2
      loop $L12
        get_local $p2
        get_local $p1
        i32.load8_u
        i32.store8
        get_local $p2
        get_local $p1
        i32.load8_u offset=1
        i32.store8 offset=1
        get_local $p2
        get_local $p1
        i32.load8_u offset=2
        i32.store8 offset=2
        get_local $p2
        get_local $p1
        i32.load8_u offset=3
        i32.store8 offset=3
        get_local $p1
        i32.const 4
        i32.add
        set_local $p1
        get_local $p2
        i32.const 4
        i32.add
        tee_local $p2
        get_local $l1
        i32.le_u
        br_if $L12
      end
    end
    get_local $p2
    get_local $l0
    i32.lt_u
    if $I13
      loop $L14
        get_local $p2
        get_local $p1
        i32.load8_u
        i32.store8
        get_local $p1
        i32.const 1
        i32.add
        set_local $p1
        get_local $p2
        i32.const 1
        i32.add
        tee_local $p2
        get_local $l0
        i32.ne
        br_if $L14
      end
    end
    get_local $p0)
  (func $memset (type $t3) (param $p0 i32) (param $p1 i32) (param $p2 i32) (result i32)
    (local $l0 i32) (local $l1 i32) (local $l2 i64)
    block $B0
      get_local $p2
      i32.eqz
      br_if $B0
      get_local $p0
      get_local $p2
      i32.add
      tee_local $l0
      i32.const 1
      i32.sub
      get_local $p1
      i32.store8
      get_local $p0
      get_local $p1
      i32.store8
      get_local $p2
      i32.const 3
      i32.lt_u
      br_if $B0
      get_local $l0
      i32.const 2
      i32.sub
      get_local $p1
      i32.store8
      get_local $p0
      get_local $p1
      i32.store8 offset=1
      get_local $l0
      i32.const 3
      i32.sub
      get_local $p1
      i32.store8
      get_local $p0
      get_local $p1
      i32.store8 offset=2
      get_local $p2
      i32.const 7
      i32.lt_u
      br_if $B0
      get_local $l0
      i32.const 4
      i32.sub
      get_local $p1
      i32.store8
      get_local $p0
      get_local $p1
      i32.store8 offset=3
      get_local $p2
      i32.const 9
      i32.lt_u
      br_if $B0
      get_local $p0
      i32.const 0
      get_local $p0
      i32.sub
      i32.const 3
      i32.and
      tee_local $l1
      i32.add
      tee_local $l0
      get_local $p1
      i32.const 255
      i32.and
      i32.const 16843009
      i32.mul
      tee_local $p1
      i32.store
      get_local $l0
      get_local $p2
      get_local $l1
      i32.sub
      i32.const -4
      i32.and
      tee_local $l1
      i32.add
      tee_local $p2
      i32.const 4
      i32.sub
      get_local $p1
      i32.store
      get_local $l1
      i32.const 9
      i32.lt_u
      br_if $B0
      get_local $l0
      get_local $p1
      i32.store offset=8
      get_local $l0
      get_local $p1
      i32.store offset=4
      get_local $p2
      i32.const 8
      i32.sub
      get_local $p1
      i32.store
      get_local $p2
      i32.const 12
      i32.sub
      get_local $p1
      i32.store
      get_local $l1
      i32.const 25
      i32.lt_u
      br_if $B0
      get_local $l0
      get_local $p1
      i32.store offset=24
      get_local $l0
      get_local $p1
      i32.store offset=20
      get_local $l0
      get_local $p1
      i32.store offset=16
      get_local $l0
      get_local $p1
      i32.store offset=12
      get_local $p2
      i32.const 16
      i32.sub
      get_local $p1
      i32.store
      get_local $p2
      i32.const 20
      i32.sub
      get_local $p1
      i32.store
      get_local $p2
      i32.const 24
      i32.sub
      get_local $p1
      i32.store
      get_local $p2
      i32.const 28
      i32.sub
      get_local $p1
      i32.store
      get_local $l1
      get_local $l0
      i32.const 4
      i32.and
      i32.const 24
      i32.or
      tee_local $l1
      i32.sub
      tee_local $p2
      i32.const 32
      i32.lt_u
      br_if $B0
      get_local $p1
      i64.extend_u/i32
      tee_local $l2
      i64.const 32
      i64.shl
      get_local $l2
      i64.or
      set_local $l2
      get_local $l0
      get_local $l1
      i32.add
      set_local $p1
      loop $L1
        get_local $p1
        get_local $l2
        i64.store offset=24
        get_local $p1
        get_local $l2
        i64.store offset=16
        get_local $p1
        get_local $l2
        i64.store offset=8
        get_local $p1
        get_local $l2
        i64.store
        get_local $p1
        i32.const 32
        i32.add
        set_local $p1
        get_local $p2
        i32.const 32
        i32.sub
        tee_local $p2
        i32.const 31
        i32.gt_u
        br_if $L1
      end
    end
    get_local $p0)
  (func $stackSave (export "stackSave") (type $t12) (result i32)
    get_global $g0)
  (func $stackRestore (export "stackRestore") (type $t1) (param $p0 i32)
    get_local $p0
    set_global $g0)
  (func $stackAlloc (export "stackAlloc") (type $t0) (param $p0 i32) (result i32)
    get_global $g0
    get_local $p0
    i32.sub
    i32.const -16
    i32.and
    tee_local $p0
    set_global $g0
    get_local $p0)
  (table $__indirect_function_table (export "__indirect_function_table") 38 38 anyfunc)
  (memory $memory (export "memory") 512 512)
  (global $g0 (mut i32) (i32.const 5250080))
  (elem (i32.const 1) $hls_decode_entry $hevc_decode_init $hevc_decode_frame $hevc_decode_free $hevc_decode_flush $hevc_v_loop_filter_chroma_8 $hevc_h_loop_filter_chroma_8 $hevc_v_loop_filter_luma_8 $hevc_h_loop_filter_luma_8 $sao_edge_filter_0_8 $sao_band_filter_0_8 $idct_4x4_dc_8 $idct_4x4_8 $transform_4x4_luma_8 $transform_rdpcm_8 $transform_skip_8 $transform_add4x4_8 $put_pcm_8 $sao_edge_filter_1_8 $idct_32x32_dc_8 $idct_16x16_dc_8 $idct_8x8_dc_8 $idct_32x32_8 $idct_16x16_8 $idct_8x8_8 $transform_add32x32_8 $transform_add16x16_8 $transform_add8x8_8 $hevc_pps_free $avcodec_default_execute2 $avcodec_default_execute $avcodec_default_get_buffer2 $av_buffer_default_free $gray_to_rgb24 $ycc_to_rgb24 $rgb_to_rgb24 $ycgco_to_rgb24)
  (data (i32.const 1026) "\01\00\01\02\00\01\02\03\01\02\03\02\03\03\00\01\00\02\01\00\03\02\01\00\03\02\01\03\02\03\00\00\01\00\01\02\00\01\02\03\00\01\02\03\04\00\01\02\03\04\05\00\01\02\03\04\05\06\00\01\02\03\04\05\06\07\01\02\03\04\05\06\07\02\03\04\05\06\07\03\04\05\06\07\04\05\06\07\05\06\07\06\07\07\00\01\00\02\01\00\03\02\01\00\04\03\02\01\00\05\04\03\02\01\00\06\05\04\03\02\01\00\07\06\05\04\03\02\01\00\07\06\05\04\03\02\01\07\06\05\04\03\02\07\06\05\04\03\07\06\05\04\07\06\05\07\06\07(-39@H")
  (data (i32.const 1200) "\1d\00\00\00\1e\00\00\00\1f\00\00\00 \00\00\00!\00\00\00!\00\00\00\22\00\00\00\22\00\00\00#\00\00\00#\00\00\00$\00\00\00$\00\00\00%\00\00\00%")
  (data (i32.const 1265) "\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01\02\03\04\05\00\01")
  (data (i32.const 1350) "\01\01\01\01\01\01\02\02\02\02\02\02\03\03\03\03\03\03\04\04\04\04\04\04\05\05\05\05\05\05\06\06\06\06\06\06\07\07\07\07\07\07\08\08\08\08\08\08\09\09\09\09\09\09\0a\0a\0a\0a\0a\0a\0b\0b\0b\0b\0b\0b\0c\0c\00\00\00\00\00\00\00\02\05\09\01\04\08\0c\03\07\0b\0e\06\0a\0d\0f\00\00\02\01\03")
  (data (i32.const 1457) "\02\05\09\0e\14\1b#\01\04\08\0d\13\1a\22*\03\07\0c\12\19!)0\06\0b\11\18 (/5\0a\10\17\1f'.49\0f\16\1e&-38<\15\1d%,27;>\1c$+16:=?\00\01\00\01\00\00\01\01")
  (data (i32.const 1537) "\01\02\03\00\01\02\03\00\01\02\03\00\01\02\03\00\00\00\00\01\01\01\01\02\02\02\02\03\03\03\03\00\01\02\03\10\11\12\13\04\05\06\07\14\15\16\17\08\09\0a\0b\18\19\1a\1b\0c\0d\0e\0f\1c\1d\1e\1f !\22#0123$%&'4567()*+89:;,-./<=>?\00\01\04\05\02\03\04\05\06\06\08\08\07\07\08\08\01\01\01\00\01\01\00\00\01\00\00\00\00\00\00\00\02\02\02\02\01\01\01\01\00\00\00\00\00\00\00\00\02\01\00\00\02\01\00\00\02\01\00\00\02\01\00\00\02\02\02\02\02\02\02\02\02\02\02\02\02\02\02\02\99\c8\8b\8d\9d\9a\9a\9a\9a\9a\9a\9a\9a\b8\9a\9a\9a\b8?\8b\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\99\8a\8ao\8d^\8a\b6\9a\8b\8b\8b\8b\8b\8bnn|}\8c\99}\7f\8cmo\8f\7foOl{?nn|}\8c\99}\7f\8cmo\8f\7foOl{?[\ab\86\8doo}nn^|l|k}\8d\b3\99}k}\8d\b3\99}k}\8d\b3\99}\8c\8b\b6\b6\98\88\98\88\99\88\8bo\88\8bo\8do\8c\5c\89\8a\8c\98\8a\8b\99J\95\5c\8bkz\98\8c\b3\a6\b6\8c\e3z\c5\8a\99\88\a7\98\98\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a")
  (data (i32.const 1911) "\99\b9k\8b~\9a\c5\b9\c9\9a\9a\9a\95\9a\8b\9a\9a\9a\98\8bnz_O?\1f\1f\99\99\99\99\8c\c6\8c\c6\a8O|\8a^\99o\95k\a7\9a\8b\8b\8b\8b\8b\8b}n^n_O}onNnoo_^l{l}n^n_O}onNnoo_^l{ly\8c=\9a\9b\9a\8b\99\8b{{?\99\a6\b7\8c\88\99\9a\a6\b7\8c\88\99\9a\a6\b7\8c\88\99\9a\aa\99{{kyky\a7\97\b7\8c\97\b7\8c\8c\8c\9a\c4\c4\a7\9a\98\a7\b6\b6\86\95\88\99y\88\89\a9\c2\a6\a7\9a\a7\89\b6k\a7[zk\a7\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a")
  (data (i32.const 2110) "\99\a0k\8b~\9a\c5\b9\c9\9a\9a\9a\86\9a\8b\9a\9a\b7\98\8b\9a\89_O?\1f\1f\99\99\99\99\a9\c6\a9\c6\a8O\e0\a7z\99o\95\5c\a7\9a\8b\8b\8b\8b\8b\8b}n|n_^}ooO}~ooOl{]}n|n_^}ooO}~ooOl{]y\8c=\9a\aa\9a\8b\99\8b{{?|\a6\b7\8c\88\99\9a\a6\b7\8c\88\99\9a\a6\b7\8c\88\99\9a\aa\99\8a\8azyzy\a7\97\b7\8c\97\b7\8c\8c\8c\9a\c4\a7\a7\9a\98\a7\b6\b6\86\95\88\99y\88z\a9\d0\a6\a7\9a\98\a7\b6k\a7[kk\a7\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a\9a")
  (data (i32.const 2336) "\06\07\08\09\0a\0b\0c\0d\0e\0f\10\11\12\14\16\18\1a\1c\1e \22$&(*,.02468:<>@")
  (data (i32.const 2402) "\01\01\01\01\01\01\01\01\01\02\02\02\02\03\03\03\03\04\04\04\05\05\06\06\07\08\09\0a\0b\0d\0e\10\12\14\16\18\1d\1e\1f !!\22\22##$$%%hevc")
  (data (i32.const 2469) "\1a\0a\01")
  (data (i32.const 2481) "\01\02\02\02\02\03\05\07\08\0a\0c\0d\0f\11\12\13\14\15\16\17\17\18\18\19\19\1a\1b\1b\1c\1c\1d\1d\1e\1f\00\07\00\00\00\01\00\00\00\00\00\00\00 \1a\15\11\0d\09\05\02\00\fe\fb\f7\f3\ef\eb\e6\e0\e6\eb\ef\f3\f7\fb\fe\00\02\05\09\0d\11\15\1a ")
  (data (i32.const 2577) "\f0\9a\f9r\fc\8a\fd\1e\fez\fe\c5\fe\00\ff\c5\fez\fe\1e\fe\8a\fdr\fc\9a\f9\00\f0\00\00@ZZZYXWUSRPNKIFC@=962.+&$\1f\19\16\12\0d\09\04\01\02\00\03\04")
  (data (i32.const 2656) "\ff\00\01\00\00\ff\00\01\ff\ff\01\01\01\ff\ff\01\10\10\10\10\11\12\15\18\10\10\10\10\11\13\16\19\10\10\11\12\14\16\19\1d\10\10\12\15\18\1b\1f$\11\11\14\18\1e#)/\12\13\16\1b#,6A\15\16\19\1f)6FX\18\19\1d$/AXs\10\10\10\10\11\12\14\18\10\10\10\11\12\14\18\19\10\10\11\12\14\18\19\1c\10\11\12\14\18\19\1c!\11\12\14\18\19\1c!)\12\14\18\19\1c!)6\14\18\19\1c!)6G\18\19\1c!)6G[\08\00\00\00\00\00\00\00\04\00\00\00\00\00\00\00\80\b0\d0\f0\80\a7\c5\e3\80\9e\bb\d8{\96\b2\cdt\8e\a9\c3o\87\a0\b9i\80\98\afdz\90\a6_t\89\9eZn\82\96Uh{\8eQcu\87M^o\80IYizEUdtBP_n>LZh;HVc8EQ^5AMY3>IU0;EP.8BL+5?H)2;E'08A%-6>#+3;!)08 '.5\1e%+2\1d#)0\1b!'-\1a\1f%+\18\1e#)\17\1c!'\16\1b %\15\1a\1e#\14\18\1d!\13\17\1b\1f\12\16\1a\1e\11\15\19\1c\10\14\17\1b\0f\13\16\19\0e\12\15\18\0e\11\14\17\0d\10\13\16\0c\0f\12\15\0c\0e\11\14\0b\0e\10\13\0b\0d\0f\12\0a\0c\0f\11\0a\0c\0e\10\09\0b\0d\0f\09\0b\0c\0e\08\0a\0c\0e\08\09\0b\0d\07\09\0b\0c\07\09\0a\0c\07\08\0a\0b\06\08\09\0b\06\07\09\0a\06\07\08\09\02\02\02\02\01\02\03\04\05\06\07\08\09\0a\0b\0c\0d\0e\0f\10\11\12\13\14\15\16\17\18\19\1a\1b\1c\1d\1e\1f !\22#$%&'()*+,-./0123456789:;<=>>?\00\00\01\02\02\04\04\05\06\07\08\09\09\0b\0b\0c\0d\0d\0f\0f\10\10\12\12\13\13\15\15\16\16\17\18\18\19\1a\1a\1b\1b\1c\1d\1d\1e\1e\1e\1f  !!!\22\22###$$$%%%&&?\00\01\01\01\01\01\01\01\01\01\01\01\01\01\01\01\02\02\02\02\02\02\02\02\02\02\02\02\02\02\02\02\03\03\03\03\03\03\03\03\04\04\04\04\04\04\04\04\05\05\05\05\06\06\06\06\07\07\07\07\08\08\08\00\00\00\01\01\02\02\02\02\03\03\03\03\03\03\03\03\04\04\04\04\04\04\04\04\04\04\04\04\04\04\04\04\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\05\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\06\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\07\00\00\00\00\00\00\00\00\03\01\01\00$8%8&8\00\00\00\00\00\00\04\00\00\00\00\00\00\00\03\01\00\10$8%8&8\00\00\00\00\00\00\05\00\00\00\00\00\00\00\03\00\00\10$8%8&8\00\00\00\00\00\00\08\00\00\00\00\00\00\00\01\00\00\00$8")
  (data (i32.const 3616) "#\00\00\00$\00\00\00%\00\00\00#\00\00\00#")
  (data (i32.const 3636) "\94\09")
  (data (i32.const 3648) "562H\220")
  (data (i32.const 3688) "\b0\11")
  (data (i32.const 3712) "\02")
  (data (i32.const 3724) "\03\00\00\00\04\00\00\00\05\00\00\00\ff\ff\ff\7f \1cP"))
