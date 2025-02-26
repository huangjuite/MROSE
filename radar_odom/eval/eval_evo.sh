odom="odom_rio.csv"
save_table="razor"
imu="razor"

ROOT=/home/rpl/Documents/RPL_radar
SEQ="1692139890_nsh_atrium 1692312182_nsh_4th 1692145970_frc 1692308112_frc_2nd 1692311541_ghc_ramp 1692923792_ghc_stair"

EVO=/tmp/evo_$save_table
APE=$EVO/ape
RPE=$EVO/rpe

mkdir $EVO
mkdir -p $APE/trans $APE/rot
mkdir -p $RPE/trans $RPE/rot

for s in $SEQ; do
    dir=$ROOT/$s
    save_name=${s#*_}
    echo $save_name

    # python3 rotate_gt.py \
    #     -d 180 \
    #     -g $dir/ground_truth/odom_vio.csv \

    python3 matching.py \
        -r $dir/result/$imu/$odom \
        -g $dir/ground_truth/odom_vio_rotate.csv \
        -og $EVO/gt_$save_name \
        -or $EVO/$save_name
    
    evo_ape tum \
        $EVO/gt_$save_name \
        $EVO/$save_name \
        -r trans_part \
        --save_result $APE/trans/$save_name.zip \
        --save_plot $APE/trans/$save_name\_ape.png \
        --no_warnings \
        --plot_mode xyz \
        --verbose \
        --align_origin

    evo_rpe tum \
        $EVO/gt_$save_name \
        $EVO/$save_name \
        -r trans_part \
        -d 1 \
        -u m \
        --save_result $RPE/trans/$save_name.zip \
        --save_plot $RPE/trans/$save_name\_rpe.png \
        --no_warnings \
        --plot_mode xyz \
        --verbose \
        --align_origin

    evo_ape tum \
        $EVO/gt_$save_name \
        $EVO/$save_name \
        -r angle_deg \
        --save_result $APE/rot/$save_name.zip \
        --save_plot $APE/rot/$save_name\_ape.png \
        --no_warnings \
        --plot_mode xyz \
        --verbose \
        --align_origin

    evo_rpe tum \
        $EVO/gt_$save_name \
        $EVO/$save_name \
        -r angle_deg \
        -d 1 \
        -u m \
        --save_result $RPE/rot/$save_name.zip \
        --save_plot $RPE/rot/$save_name\_rpe.png \
        --no_warnings \
        --plot_mode xyz \
        --verbose \
        --align_origin

done

save_result=$ROOT/results/$save_table
mkdir -p $save_result/images

evo_res $APE/trans/*.zip \
    --save_table $save_result/ape_trans.csv \
    --save_plot $save_result/images/ape_trans.png \
    --no_warnings

evo_res $APE/rot/*.zip \
    --save_table $save_result/ape_rot.csv \
    --save_plot $save_result/images/ape_rot.png \
    --no_warnings

evo_res $RPE/trans/*.zip \
    --save_table $save_result/rpe_trans.csv \
    --save_plot $save_result/images/rpe_trans.png \
    --no_warnings

evo_res $RPE/rot/*.zip \
    --save_table $save_result/rpe_rot.csv \
    --save_plot $save_result/images/rpe_rot.png \
    --no_warnings
