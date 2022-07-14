from pathlib import Path
import os

if __name__ == '__main__':
    bag_name = "car_loop_onboard"
    data_dir = Path("/media/psf/robo/bagfiles/realsense") / bag_name
    left_dir = data_dir / "infra1"
    right_dir = data_dir / "infra2"
    image_list_l = sorted(list(left_dir.glob("*.png")))
    image_list_r = sorted(list(right_dir.glob("*.png")))
    # for ll in image_list_l:
      # print(ll)
    ll = min(len(image_list_l), len(image_list_r))
    for ii in range(ll):
      img_l = image_list_l[ii]
      img_r = image_list_r[ii]
      print(int(img_l.stem[-7:]), int(img_r.stem[-7:]), abs(int(img_l.stem[-7:]) - int(img_r.stem[-7:])))
      # print(img_l.stem, img_r.stem)
