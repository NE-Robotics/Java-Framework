package utilities;

import java.io.File;
import java.io.FilenameFilter;

public class FileUtilities {
    public static String[] getFileNamesWithoutExtensions(String directoryPath) {
        File directory = new File(directoryPath);

        if (directory.isDirectory()) {
            FilenameFilter filter = (dir, name) -> name.lastIndexOf('.') > 0;

            String[] files = directory.list(filter);
            if (files != null) {
                for (int i = 0; i < files.length; i++) {
                    files[i] = removeFileExtension(files[i]);
                }
                return files;
            }
        } else {
            System.out.println("WARNING: Directory does not exist: " + directoryPath);
        }
        return null;
    }

    private static String removeFileExtension(String fileName) {
        int index = fileName.lastIndexOf('.');
        if (index > 0) {
            return fileName.substring(0, index);
        }
        return fileName;
    }
}
