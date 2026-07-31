"""Import the generated input file into Abaqus/CAE and save a CAE database."""

from __future__ import print_function

import os

from abaqus import mdb


JOB_NAME = "wide_roll_1_8"
INP_PATH = os.path.abspath(JOB_NAME + ".inp")
CAE_PATH = os.path.abspath(JOB_NAME + ".cae")


def main():
    if JOB_NAME in mdb.models:
        del mdb.models[JOB_NAME]
    mdb.ModelFromInputFile(name=JOB_NAME, inputFileName=INP_PATH)
    mdb.saveAs(pathName=CAE_PATH)
    print("Saved %s" % CAE_PATH)


if __name__ == "__main__":
    main()
