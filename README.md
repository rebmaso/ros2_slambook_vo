# ros_slambook_vo

This is my adaptation of the visual odometry code in [the SlamBook](https://github.com/gaoxiang12/slambook2). I'm using it as a baseline to modify small things and add features, such as relocalization.

## Changelog

- added relocalization

- added ros 2 support

- added opencv4 suppoort


## TODOs

- add prior to windowed BA ... this is wrong and leads to hessian singularities if not using LM

- old keyframes are stored w images landmarks ...and keep growing. bad. do not keep old images. just save descriptrors using dbow & use those descriptors to match loop kf

- add covisibility constraints for loop closure

- full loop closure w pose graph optimization (not just reloc)

- loop closure relocalization: do not add observations tying new frames to loop frames. do not modify map

- handle stuff in quques, or else just add to mapping thread, which is also triggered by each new kf

- maybe better to see why loop kf landmarks deleted and fix that.

- handle concurrency etter between reloc and windowed BA. perhaps shouldmake copy of graph.

- Use gloobal pose grraph to integrate GPS meas? Or tght integration (better, sets scale of monoslam)
