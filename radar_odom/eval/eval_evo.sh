odom="odom_rio.csv"
save_table="razor_aligned"
imu="razor"

ROOT=/home/rpl/Documents/RPL_radar
SEQ="1692139890_nsh_atrium 1692312182_nsh_4th 1692145970_frc 1692308112_frc_2nd 1692311541_ghc_ramp"

APE_TMP=/tmp/evo_ape_$save_table
RPE_TMP=/tmp/evo_rpe_$save_table
EVO_TMP=/tmp/evo_$save_table

mkdir $APE_TMP
mkdir $RPE_TMP
mkdir $EVO_TMP

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
        -og $EVO_TMP/gt_$save_name \
        -or $EVO_TMP/$save_name

    evo_ape tum \
        $EVO_TMP/gt_$save_name \
        $EVO_TMP/$save_name \
        --save_result $APE_TMP/$save_name.zip \
        --save_plot $APE_TMP/$save_name\_ape.png \
        --no_warnings \
        --plot_mode yz \
        --verbose \
        --align

    evo_rpe tum \
        $EVO_TMP/gt_$save_name \
        $EVO_TMP/$save_name \
        --save_result $RPE_TMP/$save_name.zip \
        --save_plot $RPE_TMP/$save_name\_rpe.png \
        --no_warnings \
        --plot_mode yz \
        --verbose \
        --align

done

mkdir -p $ROOT/results/images

evo_res $APE_TMP/*.zip \
    --save_table $ROOT/results/$save_table\_ape.csv \
    --save_plot $ROOT/results/images/$save_table\_ape.png \
    --no_warnings

evo_res $RPE_TMP/*.zip \
    --save_table $ROOT/results/$save_table\_rpe.csv \
    --save_plot $ROOT/results/images/$save_table\_rpe.png \
    --no_warnings
