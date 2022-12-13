import shutil
import one_sheepherd
import two_shepherds
import cv2
import os

# when sheepheard_size=24 -> article sheep positon
def main(num_of_dogs=1, sheepheard_size=40):

    # clear figs
    folder = 'figs'
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))

    # ONE DOG
    if num_of_dogs==1:
        s = one_sheepherd.Shepherd(sheepheard_size=sheepheard_size, max_steps=5000)
        s.run()
    # TWO DOGS
    else:
        s = two_shepherds.Shepherd(sheepheard_size=sheepheard_size, max_steps=5000)
        s.run()

    # plot
    image_folder = './figs'
    video_name = 'video.avi'

    images = sorted([img for img in os.listdir(image_folder) if img.endswith(".png")])
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, 0, 10, (width, height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()

if __name__ == '__main__':
    main(2, 24)

