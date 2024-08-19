import glob
import os.path as osp


def extract_subdir_list(path):
    return glob.glob(path + '/*/')


def read_files(path, extension='*.*', recursive=False):
    if path is None:
        return []
    if recursive:
        subdirs = glob.glob(path + '/*/')
        files = []
        for subdir in subdirs:
            # files+=glob.glob(subdir + '/' + extension)
            if len(extract_subdir_list(subdir)) > 0:
                files += read_files(subdir, extension, recursive=recursive)
            else:
                files += glob.glob(subdir + '/' + extension)
        return files
    else:
        return glob.glob(osp.join(path, extension))
