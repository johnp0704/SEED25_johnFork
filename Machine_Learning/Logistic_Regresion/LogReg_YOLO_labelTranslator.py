import os
import glob # Useful for finding all files matching a pattern

# ================= Configuration =================
#set paths to the YOLO data paths, and output for logisitc regression
BASE_DIR = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\YOLOTestingAnnotations\Data\labels"
OUTPUT_DIR = r"C:\UVM\SEED25_johnFork\Images\Testing Annotated\LogRegannotations"

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


if __name__ == '__main__':
    create_logistic_regression_labels()