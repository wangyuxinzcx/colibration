#include "CaptureVideo.h"

// TXT�ĵ��ַ�ת��Ϊ��д����ƥ��
std::string CaptureVideo::to_upper(const std::string& str) {
    std::string upper_str = str;
    std::transform(upper_str.begin(), upper_str.end(), upper_str.begin(), ::toupper);
    return upper_str;
}

// ����Colmap����
int CaptureVideo::execute_colmap_commands(
    const std::string& colmap_path,
    const std::string& database_path,
    const std::string& image_path,
    const std::string& output_path) {
    // ɾ���ɵ�database.db
    const char* filePath = database_path.c_str();
    if (std::FILE* file = std::fopen(filePath, "r")) {
        std::fclose(file);
        if (std::remove(filePath) == 0) {
            std::cout << "File deletion succeeded: " << filePath << std::endl;
        }
        else {
            std::cerr << "Error: File deletion failed" << filePath << std::endl;
        }
    }
    // �½�database.db
    std::string database_creator_command = colmap_path + " database_creator --database_path " + database_path;
    int return_code = std::system(database_creator_command.c_str());
    if (return_code != 0) {
        std::cerr << "Error executing database_creator command!" << std::endl;
        return return_code;
    }
    // ������ȡ
    std::string feature_extractor_command = colmap_path + " feature_extractor --database_path " + database_path + " --image_path " + image_path;
    return_code = std::system(feature_extractor_command.c_str());
    if (return_code != 0) {
        std::cerr << "Error executing feature_extractor command!" << std::endl;
        return return_code;
    }
    // ����ƥ��
    std::string exhaustive_matcher_command = colmap_path + " exhaustive_matcher --database_path " + database_path;
    return_code = std::system(exhaustive_matcher_command.c_str());
    if (return_code != 0) {
        std::cerr << "Error executing exhaustive_matcher command!" << std::endl;
        return return_code;
    }
    // ϡ���ؽ�
    std::string mapper_command = colmap_path + " mapper --database_path " + database_path + " --image_path " + image_path + " --output_path " + output_path;
    return_code = std::system(mapper_command.c_str());
    if (return_code != 0) {
        std::cerr << "Error executing mapper command!" << std::endl;
        return return_code;
    }
    // ���תΪTXT��ʽ
    std::string export_model_command = colmap_path + " model_converter --input_path " + output_path + "\\0 --output_path " + output_path + "\\0 --output_type TXT";
    return_code = std::system(export_model_command.c_str());
    if (return_code != 0) {
        std::cerr << "Error exporting model as text!" << std::endl;
        return return_code;
    }

    std::cout << "COLMAP commands executed successfully!" << std::endl;
    return 0;
}

// ����Ԫ��������ת����
Eigen::Matrix3d CaptureVideo::quaternionToRotationMatrix(double w, double x, double y, double z) {
    Eigen::Matrix3d R;

    R(0, 0) = 1 - 2 * y * y - 2 * z * z;
    R(0, 1) = 2 * x * y - 2 * w * z;
    R(0, 2) = 2 * x * z + 2 * w * y;

    R(1, 0) = 2 * x * y + 2 * w * z;
    R(1, 1) = 1 - 2 * x * x - 2 * z * z;
    R(1, 2) = 2 * y * z - 2 * w * x;

    R(2, 0) = 2 * x * z - 2 * w * y;
    R(2, 1) = 2 * y * z + 2 * w * x;
    R(2, 2) = 1 - 2 * x * x - 2 * y * y;

    return R;
}

// ��ȡ�����κ���ʵ��(������R��������T)**
std::vector<Eigen::Matrix3d> CaptureVideo::getCameraExtrinsics(const std::string& filePath, int numCameras) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file: " + filePath);
    }

    std::vector<Eigen::Matrix3d> rotationMatrices(numCameras);
    std::string line;
    int cameraIndex = 0;

    while (std::getline(file, line) && cameraIndex < numCameras) {
        line = to_upper(line);
        if (line.find(".JPG") != std::string::npos) {
            std::istringstream iss(line);
            int imgIndex, cameraId;
            double qw, qx, qy, qz, tx, ty, tz;
            std::string imgName;

            if (iss >> imgIndex >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> cameraId >> imgName) {
                Eigen::Matrix3d R = quaternionToRotationMatrix(qw, qx, qy, qz);
                rotationMatrices[cameraId - 1] = R;
                cameraIndex++;
/*
                Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
                std::cout << "Rotation Matrix for Camera " << cameraId << ":\n"
                    << rotationMatrices[cameraId - 1].format(CleanFmt) << "\n\n";
                */
            }

        }
    }

    file.close();
    return rotationMatrices;
}

// ��ȡ�����κ���ʵ��(������T��������R)
std::vector<Eigen::Vector3d> CaptureVideo::getCameraExtrinsics_T(const std::string& filePath, int numCameras) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file: " + filePath);
    }

    std::vector<Eigen::Vector3d> T(numCameras);
    std::string line;
    int cameraIndex = 0;

    while (std::getline(file, line) && cameraIndex < numCameras) {
        line = to_upper(line);
        if (line.find(".JPG") != std::string::npos) {
            std::istringstream iss(line);
            int imgIndex, cameraId;
            double qw, qx, qy, qz, tx, ty, tz;
            std::string imgName;

            if (iss >> imgIndex >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> cameraId >> imgName) {
                T[cameraId - 1] << tx, ty, tz;
                cameraIndex++;
            }
        }
    }

    file.close();
    return T;
}


// ��ȡ����ڲκ���ʵ��
std::vector<Eigen::Matrix3d> CaptureVideo::getCameraIntrinsics(const std::string& filePath, int numCameras) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file: " + filePath);
    }

    std::vector<Eigen::Matrix3d> intrinsics(numCameras);
    std::string line;
    int cameraIndex = 0;

    while (std::getline(file, line) && cameraIndex < numCameras) {
        line = to_upper(line);
        if (line.find("SIMPLE") != std::string::npos) {
            std::istringstream iss(line);
            double f, cx, cy, s;
            int h, w, cameraId;
            std::string model;

            if (iss >> cameraId >> model >> h >> w >> f >> cx >> cy >> s) {
                Eigen::Matrix3d K;
                K << f, s, cx,
                    0, f, cy,
                    0, 0, 1;

                intrinsics[cameraId - 1] = K;
                cameraIndex++;
                /*
                Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
                std::cout << "Rotation Matrix for Camera " << cameraId << ":\n"
                    << intrinsics[cameraId - 1].format(CleanFmt) << "\n\n";
*/
            }
        }
    }
    file.close();
    return intrinsics;
}

// �����������
std::vector<Eigen::Matrix3d> CaptureVideo::computeRectificationMatrices(
    int numCameras,
    const std::vector<Eigen::Matrix3d>& rotationMatrices,
    const std::vector<Eigen::Matrix3d>& intrinsics) {

    // �����������Ĵ�С�Ƿ�ƥ���������
    if (rotationMatrices.size() != numCameras || intrinsics.size() != numCameras) {
        throw std::invalid_argument("Number of cameras and size of rotationMatrices and intrinsics vectors must match");
    }

    std::vector<Eigen::Matrix3d> rectificationMatrices(numCameras);

    // �ο����
    Eigen::Matrix3d R_ref = rotationMatrices[(numCameras - 1) / 2];
    Eigen::Matrix3d K_ref = intrinsics[(numCameras - 1) / 2];

    for (int i = 0; i < numCameras; ++i) {
        Eigen::Matrix3d R = rotationMatrices[i];
        Eigen::Matrix3d K = intrinsics[i];

        rectificationMatrices[i] = K_ref * R_ref * R.transpose() * K.inverse();
        rectificationMatrices[i] /= rectificationMatrices[i](2, 2);
    }

    return rectificationMatrices;
}

std::vector<Eigen::Matrix3d> CaptureVideo::computeRectificationMatrices_2(
    int numCameras,
    const std::vector<Eigen::Matrix3d>& rotationMatrices,
    const std::vector<Eigen::Matrix3d>& intrinsics,
    const std::vector<Eigen::Vector3d>& T) {
    // �����������Ĵ�С�Ƿ�ƥ���������
    if (rotationMatrices.size() != numCameras || intrinsics.size() != numCameras) {
        throw std::invalid_argument("Number of cameras and size of rotationMatrices and intrinsics vectors must match");
    }
    std::vector<Eigen::Matrix3d> rectificationMatrices(numCameras);
    Eigen::Vector3d c1 = -rotationMatrices[0].transpose()  * T[0];
    Eigen::Vector3d c2 = -rotationMatrices[numCameras - 1].transpose() * T[numCameras - 1];
    Eigen::Vector3d r1 = (c1 - c2).normalized();
    Eigen::Vector3d r2 = (rotationMatrices[0].row(2).transpose().cross(r1)).normalized();
    Eigen::Vector3d r3 = r1.cross(r2);

    Eigen::Matrix3d R_ref ;
    Eigen::Matrix3d K_ref ;

    R_ref << r1.transpose(), r2.transpose(), r3.transpose();
    K_ref << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    for (int i = 0; i < numCameras; i++) {
        K_ref += intrinsics[i];
    }
    K_ref /= numCameras;

    for (int i = 0; i < numCameras; ++i) {
        Eigen::Matrix3d R = rotationMatrices[i];
        Eigen::Matrix3d K = intrinsics[i];

        rectificationMatrices[i] = K_ref * R_ref * R.transpose() * K.inverse();
        rectificationMatrices[i] /= rectificationMatrices[i](2, 2);
    }

    return rectificationMatrices;

}

// ���浽xml�ļ�
void CaptureVideo::saveToXML(const std::vector<Eigen::Matrix3d>& rectificationMatrices, const std::string& outputFilePath) {
    std::ofstream file(outputFilePath);
    if (!file.is_open()) {
        throw std::runtime_error("�޷����ļ�: " + outputFilePath);
    }

    file << "<?xml version=\"1.0\"?>\n";
    file << "<opencv_storage>\n";

    for (size_t i = 0; i < rectificationMatrices.size(); ++i) {
        file << "<H_mat" << i << " type_id=\"opencv-matrix\">\n";
        file << "  <rows>3</rows>\n";
        file << "  <cols>3</cols>\n";
        file << "  <dt>d</dt>\n";
        file << "  <data>\n";
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                file << "    " << rectificationMatrices[i](r, c);
                if (r != 2 || c != 2)
                    file << " ";
            }
            file << "\n";
        }
        file << " </data>\n";
        file << "</H_mat" << i << ">\n";
    }

    file << "</opencv_storage>\n";
    file.close();
}

void CaptureVideo::Calibrate() {
    int numCamera = 5;
    std::string database_path = "D:\\practice\\test_scene_2\\database.db";
    std::string image_path = "D:\\practice\\test_scene_2\\images";
    std::string output_path = "D:\\practice\\test_scene_2\\output";
    std::string colmap_path = "D:\\����\\COLMAP-3.9.1-windows-cuda\\COLMAP-3.9.1-windows-cuda\\colmap.bat";
    //int result = execute_colmap_commands(colmap_path, database_path, image_path, output_path);
    int result = 0;
    if (result == 0) {
        std::vector<Eigen::Matrix3d> Extrinsics = getCameraExtrinsics(output_path + "\\0\\images.txt", numCamera);
        //std::vector<Eigen::Vector3d> T = getCameraExtrinsics_T(output_path + "\\0\\images.txt", numCamera);
        std::vector<Eigen::Matrix3d> Intrinsics = getCameraIntrinsics(output_path + "\\0\\cameras.txt", numCamera);
        std::vector<Eigen::Matrix3d> Rectification = computeRectificationMatrices(numCamera, Extrinsics, Intrinsics);
        //std::vector<Eigen::Matrix3d> Rectification = computeRectificationMatrices_2(numCamera, Extrinsics, Intrinsics, T);
        saveToXML(Rectification, output_path + "H_mat_2.xml");
    }
}


// �ݹ�ɾ���ļ��м�����������
bool deleteFolder(const std::wstring& folderPath) {
    WIN32_FIND_DATAW findFileData; // ���ڴ洢���ҵ����ļ����ļ��е������Ϣ
    // �����ļ����еĵ�һ���ļ����ļ���
    HANDLE hFind = FindFirstFileW((folderPath + L"\\*").c_str(), &findFileData);

    if (hFind == INVALID_HANDLE_VALUE) {
        // �������ʧ�ܣ����� false
        return false;
    }

    do {
        // ������ǰ�ļ����ļ��е�����·��
        const std::wstring fileOrFolder = folderPath + L"\\" + findFileData.cFileName;
        if (findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            // ������ļ���
            if (wcscmp(findFileData.cFileName, L".") != 0 && wcscmp(findFileData.cFileName, L"..") != 0) {
                // �ų���ǰĿ¼ "." �͸�Ŀ¼ ".."
                deleteFolder(fileOrFolder); // �ݹ�ɾ�����ļ���
            }
        }
        else {
            // ������ļ�
            DeleteFileW(fileOrFolder.c_str()); // ɾ���ļ�
        }
    } while (FindNextFileW(hFind, &findFileData) != 0); // ������һ���ļ����ļ���

    FindClose(hFind); // �رղ��Ҿ��
    return RemoveDirectoryW(folderPath.c_str()); // ɾ�����ļ��в����ؽ��
}

// �½��ļ��У�����ļ����Ѵ��ڣ���ո��ļ���
bool CreateFolder(const std::wstring& folderPath) {
    // ����ļ����Ƿ��Ѿ�����
    if (GetFileAttributesW(folderPath.c_str()) != INVALID_FILE_ATTRIBUTES) {
        // ����ļ��д��ڣ�ɾ����
        if (deleteFolder(folderPath)) {
            std::wcout << L"�Ѵ��ڵ��ļ�����ɾ��: " << folderPath << std::endl;
        }
        else {
            std::wcout << L"ɾ���ļ���ʧ��: " << folderPath << std::endl;
            return 1; // �˳����򣬱�����������ļ��в���
        }
    }

    // �����ļ���
    if (CreateDirectoryW(folderPath.c_str(), NULL)) {
        std::wcout << L"�ļ��д����ɹ�: " << folderPath << std::endl;
        return TRUE;
    }
    else {
        std::wcout << L"�ļ��д���ʧ��: " << folderPath << std::endl;
        return FALSE;
    }
}

int main() {
    CaptureVideo::Calibrate();
}