#!/bin/bash

# ========== Params ==========
# apartment_dynamic_object, tesse_cd_office_dynamic_object
config=$(rospack find khronos_eval)/config/ground_truth/tesse_cd_office_dynamic_objects.yaml

# ========== Autogenerated arguments ==========
exec_dir=$(catkin_find --first-only --without-underlays --libexec khronos_eval)

# ========== Build the GT files ==========
eval $exec_dir/create_dynamic_object_gt $config
