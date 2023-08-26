odom="odom_rio.csv"

ROOT=/home/rpl/Documents/RPL_radar

APE_TMP=/tmp/evo_ape
RPE_TMP=/tmp/evo_rpe


mkdir -p $ROOT/results/images

cd /home/rpl/evo/evo
python3 main_box.py \
    $APE_TMP\_razor_aligned/*.zip \
    $APE_TMP\_razor_r0/*.zip \
    $APE_TMP\_razor_r1/*.zip \
    --save_plot $ROOT/results/images/all_ape.png \
    --no_warnings \
    -p

python3 main_box.py \
    $RPE_TMP\_razor_aligned/*.zip \
    $RPE_TMP\_razor_r0/*.zip \
    $RPE_TMP\_razor_r1/*.zip \
    --save_plot $ROOT/results/images/all_rpe.png \
    --no_warnings \
    -p