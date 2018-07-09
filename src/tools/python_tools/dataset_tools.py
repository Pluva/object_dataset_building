#!/usr/bin/env python


"""
    File containing python tools to handle the datasets produced with the object_dataset_building functions.
"""



#python
import os
import argparse



def remove_small_images(min_size_x, min_size_y=-1, dir_path=''):
    """
        Browse a directory of images, removes the one that are smaller than the given mininum size.

        :param min_size_x: Minimum width of the image.
        :param min_size_Y: Minimum heidht of the image, equal to x if not provided. 
        :param dir_path: Absolute path to the directory to browse.
        :type min_size_x: int
        :type min_size_y: int
        :type dir_path: string
    """

    white_listed_format = ['jpg', 'png']

    if min_size_y == -1:
        min_size_y = min_size_x

    is_valid = False

    with os.scandir(path=dir_path) as it:
        for entry in it:
            if not entry.name.startswith('.') and entry.is_file():
                is_valid = False
                for extension in white_listed_format:
                    if entry.name.endswith('.'+extension):
                        is_valid = True

                if is_valid:
                    print(entry.name)

    return


def create_dataset_root_file(dir_path=''):
    """
        Browse and create a root_file in a dataset file.
        The root file contains as many line as the number of images in the folder.
        Each line being: "file_name";"label"
        The resulting pairs are saved in a newly created file, named images_labels + '.drf'.

        :param dir_path: Absolute path to the directory to browse.
        :type dir_path: string
    """



    return


if __name__ == '__main__':


    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--mode', help='Script mode', choices=['rsi', 'cdrf'])
    parser.add_argument('-d', '--dir', help='Absolute path to a directory.')

    parser.add_argument('-s', '--size', help='Minimum size of the images, smaller being removed.')

    args = parser.parse_args()

    if args.mode == 'rsi':
        if not(args.size):
            parser.error('You need to specify the minimum size. [-s, --size] option.')          
        elif not(args.dir):
            parser.error('You need to specify the target directory. [-d, --dir] option.')           
        else:
            print('Entering rsf (Remove Small Images) mode.')
            remove_small_images(dir_path=args.dir, min_size_x=args.size)

    elif args.mode == 'cdrf':
        if not(args.dir):
            parser.error('You need to specify the target directory. [-d, --dir] option.')
        else:
            print('Entering cdrf mode (Create Dataset Root File).')
            create_dataset_root_file(dir_path=args.dir)