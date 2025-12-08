import os
import glob

#=================Config=================
#fet to folder containing YOLO 'images' and 'labels'
BASE_DIR = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\YOLOTestingAnnotations\Data"

#output folder
OUTPUT_DIR = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\LogRegannotations"

#sets we are looking for
DATA_SETS = ['train', 'val', 'test']


#=================Helper Functions=================
def get_label_from_yolo(txt_path):
    '''
    Reads a YOLO text file and determines if a dandelion (class 0) is present.
    Returns: 1 if class 0 is found, 0 otherwise.
    '''
    if not os.path.exists(txt_path):
        return 0

    try:
        with open(txt_path, 'r') as f:
            lines = f.readlines()

        for line in lines:
            parts = line.strip().split()
            if len(parts) > 0:
                class_id = int(parts[0])
                if class_id == 0:
                    return 1
        return 0

    except Exception as e:
        print(f'Error reading {txt_path}: {e}')
        return 0


#=================Main=================
def create_flat_logistic_regression_files():
    '''
    Creates train_image_paths.txt, train_binary_labels.txt, etc.
    '''
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print(f'\nReading YOLO dataset from: {BASE_DIR}')
    print(f'Outputting files to: {OUTPUT_DIR}\n')

    for split in DATA_SETS:
        #YOLO folders
        image_dir = os.path.join(BASE_DIR, 'images', split)
        label_dir = os.path.join(BASE_DIR, 'labels', split)

        #output file paths
        out_img_paths = os.path.join(OUTPUT_DIR, f'{split}_image_paths.txt')
        out_labels = os.path.join(OUTPUT_DIR, f'{split}_binary_labels.txt')

        image_files = glob.glob(os.path.join(image_dir, '*.png'))

        all_image_paths = []
        all_binary_labels = []

        print(f'Processing {split}: {len(image_files)} images found.')

        for image_path in image_files:
            filename = os.path.splitext(os.path.basename(image_path))[0]
            yolo_txt = os.path.join(label_dir, f'{filename}.txt')

            label = get_label_from_yolo(yolo_txt)

            all_image_paths.append(image_path)
            all_binary_labels.append(str(label))

        with open(out_img_paths, 'w') as f:
            f.write('\n'.join(all_image_paths))

        with open(out_labels, 'w') as f:
            f.write('\n'.join(all_binary_labels))

        print(f'  Saved: {out_img_paths}')
        print(f'  Saved: {out_labels}')


#=================Run=================
if __name__ == '__main__':
    create_flat_logistic_regression_files()
