#! /usr/bin/python3

import spark_dsg as dsg
import argparse
import os

"""
Turns DSG files from json to binary files.
If used on a directory, will (optionally recursively) binarize all json DSG files in the directory.
"""


def main(args):
    # Optional: Overwrite args.
    # args.path = "/mnt/c/Users/DerFu/Documents/khronos"
    # args.cleanup = True

    if os.path.isdir(args.path):
        # Get all files in the directory.
        print(f"Searching for DSGs in {args.path}...")
        files = []
        if args.recursive:
            for root, dirs, fs in os.walk(args.path):
                for f in fs:
                    if not f.endswith(".json"):
                        continue
                    files.append(os.path.join(root, f))
        else:
            files = os.listdir(args.path)
            files = [os.path.join(args.path, f) for f in files if f.endswith(".json")]
    elif os.path.isfile(args.path):
        files = [args.path]
    else:
        raise ValueError(f"Invalid path {args.path}")

    files = sorted(files)

    num_binarized = 0
    num_skipped = 0
    num_failed = 0
    num_files = len(files)
    print(f"Found {num_files} candidate files to binarize. Processing...")

    # Binarize each file.
    for i, file in enumerate(files):
        try:
            graph = dsg.DynamicSceneGraph.load(str(file))
        except:
            num_skipped += 1
            print(f"  {i+1}/{num_files} : Skipped : {file}")
            continue
        new_name = file.replace(".json", ".sparkdsg")
        try:
            graph.save(str(new_name), graph.has_mesh())
        except:
            num_failed += 1
            print(f"  {i+1}/{num_files} : Failed : {file}")
            continue
        num_binarized += 1
        print(f"  {i+1}/{num_files} : Success : {file}")
        if args.cleanup:
            os.remove(file)

    print(
        f"Binarized {num_binarized} files. Skipped {num_skipped} files. Failed to binarize {num_failed} files."
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=str, help="Directory to binarize DSGs in")
    parser.add_argument(
        "--recursive", type=bool, default=True, help="Whether to check subdirectories"
    )
    parser.add_argument(
        "--cleanup", type=bool, default=False, help="Whether to remove the old files"
    )
    args = parser.parse_args()
    main(args)
