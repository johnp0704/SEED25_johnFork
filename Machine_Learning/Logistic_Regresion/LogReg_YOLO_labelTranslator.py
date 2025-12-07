import os
import glob # Useful for finding all files matching a pattern

# ================= Configuration =================
#set to folder containing 'images' and 'labels' folders for YOLO  Data
BASE_DIR = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\YOLOTestingAnnotations\Data"

#root folder paths
OUTPUT_ROOT = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\LogRegannotations"

#set final output directory to include the 'labels' subfolder
OUTPUT_DIR = os.path.join(OUTPUT_ROOT, "labels")

#sets we are looking for
DATA_SETS = ['train', 'val', 'test']

# =================Helper Function=================
def get_label_from_yolo(txt_path):
    '''
    Reads a YOLO text file and determines if a dandelion (class 0) is present.
    Returns: 1 if class 0 is found, 0 otherwise.
    '''
    if not os.path.exists(txt_path):
        return 0 # no label file implies no dandelion (shouldn't happen but just in case)
    
    try:
        with open(txt_path, 'r') as f:
            lines = f.readlines()
            
        for line in lines:
            parts = line.strip().split()
            if len(parts) > 0:
                #first number in the YOLO label file is the classification id
                class_id = int(parts[0]) 
                
                #check for class 0 (yes dandelion)
                if class_id == 0:
                    return 1 #yes dandelion (inconsistent, I know)
                    
        return 0 #file exists but no dandelion labeled
        
    except Exception as e:
        #error handling (file corruption or github errors, which there are a lot of)
        print(f'Error reading {txt_path}: {e}') 
        return 0

# =================Main=================
def create_logistic_regression_labels():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    print(f'Reading from: {BASE_DIR}')
    
    for split in DATA_SETS:
        #define input/output folders for the current split
        image_dir = os.path.join(BASE_DIR, 'images', split)
        label_dir = os.path.join(BASE_DIR, 'labels', split)
        
        #define output files for the current split
        output_images_path = os.path.join(OUTPUT_DIR, f'{split}_image_paths.txt')
        output_labels_path = os.path.join(OUTPUT_DIR, f'{split}_binary_labels.txt')
        
        # Use glob to find all images (credit to Gemini 3 for suggesting using glob)
        image_files = glob.glob(os.path.join(image_dir, '*.png')) 
        
        #store data
        all_image_paths = []
        all_binary_labels = []

        print(f'Processing {split} set: ({len(image_files)} images)')
        
        for image_path in image_files:
            #get file name to associate properly
            filename_with_ext = os.path.basename(image_path)
            filename = os.path.splitext(filename_with_ext)[0] #remove extension
            
            #make path of YOLO label
            yolo_label_path = os.path.join(label_dir, f'{filename}.txt')
            
            #get binary classification
            binary_label = get_label_from_yolo(yolo_label_path)
            
            #save results
            all_image_paths.append(image_path)
            all_binary_labels.append(str(binary_label))

        # 5. Write the final files
        with open(output_images_path, 'w') as f:
            f.write('\n'.join(all_image_paths))
            
        with open(output_labels_path, 'w') as f:
            f.write('\n'.join(all_binary_labels))
            
        print(f'Successfully generated {len(all_image_paths)} labels for {split} set.')

    print(f'\nFiles saved in: {OUTPUT_DIR}')


# =================File Organization=================
def organize_files(output_root):
    """
    Moves the content of the generated single files (e.g., train_binary_labels.txt)
    into the desired separate structure (e.g., /labels/train/binary_labels.txt).
    """
    DATA_SETS = ['train', 'val', 'test']
    
    #current location of files
    temp_label_dir = os.path.join(output_root, "labels")
    
    for split in DATA_SETS:
        #temporary input files
        temp_image_path_file = os.path.join(temp_label_dir, f"{split}_image_paths.txt")
        temp_label_file = os.path.join(temp_label_dir, f"{split}_binary_labels.txt")

        #error handling courtesy of Gemini 3
        if not os.path.exists(temp_label_file) or not os.path.exists(temp_image_path_file):
            print(f"Warning: Temporary files for {split} not found. Skipping organization.")
            continue

        #destination folders
        final_label_dir = os.path.join(temp_label_dir, split)
        final_image_dir = os.path.join(output_root, "images", split)

        #make destination folders
        os.makedirs(final_label_dir, exist_ok=True)
        os.makedirs(final_image_dir, exist_ok=True)
        
        #move labels
        final_label_path = os.path.join(final_label_dir, "binary_labels.txt")
        os.rename(temp_label_file, final_label_path)
        print(f"Labels moved to: {final_label_path}")
        
        #move image paths
        final_image_path = os.path.join(final_image_dir, "image_paths.txt")
        os.rename(temp_image_path_file, final_image_path)
        print(f"Image Paths moved to: {final_image_path}")

    print("File organization complete")
    
# ================= Main Execution Block Amendment =================

if __name__ == '__main__':
    create_logistic_regression_labels() 
    organize_files(OUTPUT_ROOT)