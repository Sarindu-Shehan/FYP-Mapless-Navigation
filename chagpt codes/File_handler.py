import psutil
from six.moves import cPickle as pickle
import time
import os
import shutil

class File_handler(object):

    def is_using(self,fpath):
        for proc in psutil.process_iter():
            try:
                for item in proc.open_files():
                    if fpath == item.path:
                        return True
            except Exception:
                pass
        return False


    def load_object(self, path, file_name):
        data = None
        if path == "" : file = file_name
        else : file = path + file_name
        while self.is_using(file):
            time.sleep(0.1)
        try:
            with open(file, 'rb') as f:
                data = pickle.load(f)
        except Exception as e:
            print('Unable to process data from', file, ':', e)
            pass
        return data


    def save_object(self,path, file_name, data):
        if path == "": file = file_name
        else: file = path + file_name
        while self.is_using(file):
            time.sleep(0.1)
        try:
            f = open(file, 'wb')
            pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
            f.close()
        except Exception as e:
            print('Unable to save data to', file, ':', e)
            pass

    def empty_directory(self,path):
        if os.path.exists(path):
            shutil.rmtree(path)
        os.mkdir(path)

    def write_to_a_text_file(self,path, nature_of_writing,content):
        with open(path, nature_of_writing) as fh:
            fh.write(str(content)+'\n')
