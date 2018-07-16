#!/usr/bin/env python


"""
    Source file of python tools to handle the datasets produced with the object_dataset_building functions.
    _move_file_to_subdir:
    _test_treshold:
    analyse_images_dataset:
    relocate_images:
    relocate_small_images:
    relocate_big_images:
    create_dataset_root_file:
"""



#python
import os
import argparse

#Pillow
from PIL import Image





def _move_file_to_subdir(fname, olddir, newdir):
    """
        Utilitary function to move a file to a new destination.

        :param file: Name of the file to move.
        :param olddir: Absolute path to the dir containing the file.
        :param newdir: Absolute path to the dir in which to move the file.
        :type file: string
        :type olddir: string
        :type newdir: string
    """

    if os.path.isfile(os.path.join(olddir, fname)):
        os.rename(os.path.join(olddir, fname), os.path.join(newdir, fname))

    return

def _test_treshold(x, y, t_x, t_y, mode):
    """
        Test x, y against the given treshold regarding the given mode.
        Return (x mode t_x OR y mode t_y)
        :param x: width to compare
        :param y: height to compare
        :param t_x: width treshold
        :param t_y: height treshold
        :param mode: Comparison mode, accepted are [<, <=, >, >=]
        :return: The comparison value.
        :type x: int
        :type y: int
        :type x_t: int
        :type y_t: int
        :type mode: string
        :rtype: boolean
    """

    if mode == '<':
        ret = (x < t_x) or (y < t_y)
    elif mode == '<=':
        ret = (x <= t_x) or (y <= t_y)
    elif mode == '>':
        ret = (x > t_x) or (y > t_y)
    elif mode == '>=':
        ret = (x >= t_x) or (y >= t_y)
    else:
        raise ValueError('Incorrect value for "mode" parameter, accepted values are ["<", "<=", ">",">="].')

    return ret


def analyse_images_dataset(src_path):
    """
        Analyse and returns very simple data about the given dataset of images.
        [nb_images, avg_size, min_size, max_size]

        :param src_path: Absolute path to the directory to analyse.
        :param verbose: Verbose behavior.
        :return: Statistical data.
        :type src_path: string
        :type verbose: boolean
        :return: Statistical data.
        :rtype: [int, float, int, int]
    """

    white_listed_format = ['jpg', 'png']
    nb_images = 0
    tot_size = 0.0
    min_size = 200000 # good enough for 2d pictures
    max_size = 0

    with os.scandir(path=src_path) as it:
        for entry in it:
            if not entry.name.startswith('.') and entry.is_file():
                is_valid = False
                for extension in white_listed_format:
                    if entry.name.endswith('.'+extension):
                        is_valid = True

                if is_valid:
                    nb_images += 1
                    image = Image.open(src_path + entry.name)
                    x, y = image.size
                    if x > max_size: max_size=x
                    if x < min_size: min_size=x
                    tot_size += x

    return (nb_images, tot_size/nb_images, min_size, max_size)


def relocate_images(treshold_x, treshold_y, src_path, target_path, mode='<', verbose=False):
    """
        Browse a directory of images, relocates the one that respect the given rule and treshold.
        For instance if treshold_x=20 and mode='<', all images that have width < to 20 pixels will be relocated.

        :param treshold_x: treshold over the width value.
        :param treshold_y: treshold over the height value.
        :param src_path: Absolute path of image sources.
        :param target_path: Absolute path to the relocating folder.
        :param mode: Comparison mode [default='<', '<=', '>', '>='].
        :param verbose: Display infos during process [default=False, True].
        :return: The number of images processed, and the number of images relocated.
        :type treshold_x: int
        :type treshold_y: int
        :type src_path: string
        :type target_path: string
        :type mode: string
        :type verbose: bool
        :rtype: (int, int)
    """

    if verbose: print('Starting relocate_images process with parameters x={}, y={}, src={}, target={}, mode={}.'.format(
        treshold_x, treshold_y, src_path, target_path, mode))

    white_listed_format = ['jpg', 'png']

    if treshold_y == -1:
        treshold_y = treshold_x

    nb_removed = 0
    nb_processed = 0

    # Create folder in which to discard rejected images
    if os.path.isdir(target_path) and verbose:
        print('WARNING: Target directory {} already exists, process may result in data being overwriten.'.format(target_path))
    else:
        os.mkdir(target_path)



    with os.scandir(path=src_path) as it:
        for entry in it:
            if not entry.name.startswith('.') and entry.is_file():
                is_valid = False
                for extension in white_listed_format:
                    if entry.name.endswith('.'+extension):
                        is_valid = True

                if is_valid:
                    nb_processed += 1
                    image = Image.open(os.path.join(src_path, entry.name))
                    x, y = image.size
                    
                    if _test_treshold(x, y, treshold_x, treshold_y, mode):
                        if verbose:
                            print("Relocating file: {}, size({},{}).".format(entry.name, x, y))
                            print('---- From {}, to {}'.format(src_path, target_path))

                        _move_file_to_subdir(fname=entry.name, olddir=src_path, newdir=target_path)
                        nb_removed += 1

    if verbose: print("Process result: processed={}, relocated={}.".format(nb_processed, nb_removed))

    return (nb_processed, nb_removed)


def relocate_small_images(dir_path, tre_x, tre_y=-1, storage_name='_storage', verbose=False):
    """
        Browse the dir_path directory.
        Any images that are stricly smaller than the given treshold are relocated in the dir_path+storage_name folder.

        :param tre_x: Minimum width of the image.
        :param tre_Y: Minimum heidht of the image, equal to x if not provided. 
        :param dir_path: Absolute path to the directory to browse.
        :param storage_name: Name of the subfolder in which to store the images.
        :return: The number of processed images and relocated images.
        :type tre_x: int
        :type tre_y: int
        :type dir_path: string
        :type storage_name: string
        :rtype: (int, int)
    """
    root = dir_path
    storage = os.path.join(dir_path, storage_name)
    return relocate_images(treshold_x=tre_x, treshold_y=tre_y, src_path=root, target_path=storage, mode='<', verbose=verbose)

def relocate_big_images(dir_path, tre_x, tre_y=-1, storage_name='_storage', verbose=False):
    """
        Browse the storage_name dir from the dir_path directory.
        Any images bigger or equal to the treshold size given are relocated in dir_path.

        :param tre_x: Minimum width of the image.
        :param tre_Y: Minimum heidht of the image, equal to x if not provided. 
        :param dir_path: Absolute path to the directory to browse.
        :param storage_name: Name of the subfolder from which to extract the images.
        :return: The number of processed images and relocated images.
        :type tre_x: int
        :type tre_y: int
        :type dir_path: string
        :type storage_name: string
        :rtype: (int, int)
    """
    root = dir_path
    storage = os.path.join(dir_path, storage_name)
    return relocate_images(treshold_x=tre_x, treshold_y=tre_y, src_path=storage, target_path=root, mode='>=', verbose=verbose)


def _process_image_label(img_name):
    """
        Process the formatted image name into name and label.
        See naming convention format in dataset_builder.cpp source file.
        For this specific case, files are formatted as follow:
            '_processed_imgID_patchID_patchLABEL_color.jpg'

        :param img_name: Name of the image.
        :return: The label associated with this image.
        :type img_name: string
        :rtype: int
    """
    return img_name.split('_')[4]

def create_dataset_root_file(dir_path, file_name='images_labels', verbose=False):
    """
        Browse a dataset folder, containing labeled images, and create a root_file.
        The root file contains as many line as the number of images in the folder.
        Each line being: file_name;label
        The resulting pairs are saved in a newly created file, named images_labels + '.drf'.

        :param dir_path: Absolute path to the directory to browse.
        :param file_name: Name of the file to create [default='images_labels'].
        :type dir_path: string
        :type file_name: string
    """

    white_listed_format = ['jpg', 'png']

    extension = '.txt'
    ofile_path = os.path.join(dir_path, file_name + extension)

    if os.path.isfile(ofile_path) and verbose:
        print('WARNING: File "{}" already exists, process will overwrite it.'.format(ofile_path))


    ofile = open(ofile_path, mode='w')

    nb_processed = 0
    with os.scandir(path=dir_path) as it:
        for entry in it:
            if not entry.name.startswith('.') and entry.is_file():
                is_valid = False
                for extension in white_listed_format:
                    if entry.name.endswith('.'+extension):
                        is_valid = True

                if is_valid:
                    nb_processed += 1
                    ofile.write('{};{}\n'.format(entry.name, _process_image_label(entry.name)))

    if verbose:
            print("Process result: processed={}.".format(nb_processed))

    ofile.close()

    return


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Set of tools to post process data generated by Leni Le Goff\'s relevance map.')
    parser.add_argument('-m', '--mode', type=str, help='Script mode: RelocateSmallImages, RelocateBigImages, CreateDatasetRootFile.', choices=['rsi', 'rbi', 'cdrf'])
    parser.add_argument('-d', '--dir', type=str, help='Absolute path to a directory.', required=True)
    parser.add_argument('-s', '--size', nargs=2, type=int, help='Treshold shape of the images to be relocated.')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print out info during process.')
    parser.add_argument('--storage', default='_storage', help='Name of the folder in which are stored the relocated images.')


    args = parser.parse_args()
    # Set default parameters


    if args.mode == 'rsi':
        # Relocate Small Images from given folder into DEFAULT_store_name
        if not(args.size):
            parser.error('You need to specify the minimum size. [-s, --size] option.')          
        elif not(args.dir):
            parser.error('You need to specify the target directory. [-d, --dir] option.')           
        else:
            print('Entering rsf (Remove Small Images) mode.')
            relocate_small_images(tre_x=args.size[0], tre_y=args.size[1], dir_path=args.dir, storage_name=args.storage, verbose=args.verbose)
           
    elif args.mode == 'rbi':
        # Relocate Big Images from DEFAULT_store_name into given folder
        if not(args.size):
            parser.error('You need to specify the minimum size. [-s, --size] option.')          
        elif not(args.dir):
            parser.error('You need to specify the target directory. [-d, --dir] option.')           
        else:
            print('Entering rsf (Remove Small Images) mode.')
            relocate_big_images(tre_x=args.size[0], tre_y=args.size[1], dir_path=args.dir, storage_name=args.storage, verbose=args.verbose)

    elif args.mode == 'cdrf':
        if not(args.dir):
            parser.error('You need to specify the target directory. [-d, --dir] option.')
        else:
            print('Entering cdrf mode (Create Dataset Root File).')
            create_dataset_root_file(dir_path=args.dir)
