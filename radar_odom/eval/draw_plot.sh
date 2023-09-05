
ROOT=/home/rpl/Documents/RPL_radar

APE_TMP=/tmp/evo_ape
RPE_TMP=/tmp/evo_rpe


# cd /home/rpl/evo/evo
# python3 main_box.py \
#     /tmp/evo_razor/ape/trans/*.zip \
#     /tmp/evo_razor_r0/ape/trans/*.zip \
#     /tmp/evo_razor_r1/ape/trans/*.zip \
#     --save_plot /tmp/ape_trans.png \
#     --no_warnings \
#     -p

# python3 main_box.py \
#     /tmp/evo_razor/ape/rot/*.zip \
#     /tmp/evo_razor_r0/ape/rot/*.zip \
#     /tmp/evo_razor_r1/ape/rot/*.zip \
#     --save_plot /tmp/ape_rot.png \
#     --no_warnings \
#     -p

# python3 main_box.py \
#     /tmp/evo_razor/rpe/trans/*.zip \
#     /tmp/evo_razor_r0/rpe/trans/*.zip \
#     /tmp/evo_razor_r1/rpe/trans/*.zip \
#     --save_plot /tmp/rpe_trans.png \
#     --no_warnings \
#     -p

# python3 main_box.py \
#     /tmp/evo_razor/rpe/rot/*.zip \
#     /tmp/evo_razor_r0/rpe/rot/*.zip \
#     /tmp/evo_razor_r1/rpe/rot/*.zip \
#     --save_plot /tmp/rpe_rot.png \
#     --no_warnings \
#     -p

cd /home/rpl/evo/evo
python3 main_box_2.py \
    /tmp/evo_razor/ape/trans/*.zip \
    /tmp/evo_epson/ape/trans/*.zip \
    --save_plot /tmp/ape_trans.png \
    --no_warnings 


python3 main_box_2.py \
    /tmp/evo_razor/ape/rot/*.zip \
    /tmp/evo_epson/ape/trans/*.zip \
    --save_plot /tmp/ape_rot.png \
    --no_warnings 


python3 main_box_2.py \
    /tmp/evo_razor/rpe/trans/*.zip \
    /tmp/evo_epson/rpe/trans/*.zip \
    --save_plot /tmp/rpe_trans.png \
    --no_warnings 


python3 main_box_2.py \
    /tmp/evo_razor/rpe/rot/*.zip \
    /tmp/evo_epson/rpe/rot/*.zip \
    --save_plot /tmp/rpe_rot.png \
    --no_warnings 
