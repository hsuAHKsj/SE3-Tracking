
#include <hansDriver/HR_Pro.h>
#include <hansDriver/conciseController.h>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <iomanip>
#include <vector>

class csvReader
{

public:
	// �ļ�·�������������ʸ��
	// ֻ�������һ�µ���
	bool fileDataRead(string filePath, int columnCounts, EcRealVectorVector& out)
	{
		EcRealVector rowData(columnCounts);

		ifstream iFile(filePath);

		string row;
		vector<string> data;
		int typeChoose;

		bool ret = true;

		if (!iFile.good())
		{
			std::cout << "file did not import." << std::endl;
			return false;
		}

		// ɾ����ǰ����
		getline(iFile, row);
		getline(iFile, row);

		while (iFile.good())
		{
			getline(iFile, row);
			splitCSVDataLine(row, ",", data);
			if (data.size() < 5)
				continue;
			//if (data.size() != columnCounts)
			//	continue;
			for (int ii = 0; ii < columnCounts; ii++)
			{
				rowData[ii] = atof(data[ii].c_str());
			}
			out.push_back(rowData);
		}

		return ret;
	}

	void writeJoints(std::string path, EcRealVectorVector& outJoint)
	{
		ofstream oFileM;
		oFileM.open(path, ios::out | ios::trunc);

		for (int i = 0; i < outJoint.size(); i++)
		{
			//std::cout << std::setprecision(12) << outJoint[i][0] << "," << outJoint[i][1] << "," << outJoint[i][2] << "," << outJoint[i][3] << "," << outJoint[i][4] << "," << outJoint[i][5] << std::endl;
			oFileM << std::setprecision(12) << outJoint[i][0] << "," << outJoint[i][1] << "," << outJoint[i][2] << "," << outJoint[i][3] << "," << outJoint[i][4] << "," << outJoint[i][5] << std::endl;
		}

		oFileM.close();
	}

	void writeTime(std::string path, std::vector<double>& Time_v)
	{
		ofstream oFileM;
		oFileM.open(path, ios::out | ios::trunc);

		for (int i = 0; i < Time_v.size(); i++)
		{
			oFileM << std::setprecision(12) << Time_v[i] << endl;
		}

		oFileM.close();
	};

private:
	void splitCSVDataLine(string str, string separator, vector<string>& result)
	{
		result.clear();
		int cutAt;
		while ((cutAt = str.find_first_of(separator)) != str.npos)
		{
			if (cutAt > 0)
			{
				result.push_back(str.substr(0, cutAt));
			}
			str = str.substr(cutAt + 1);
		}
		if (str.length() > 0)
		{
			result.push_back(str);
		}
	}

};


int main()
{
	HRIF_Connect(0, "10.0.0.46", 10003);

	if (HRIF_IsConnected(0))
	{
		std::cout << "Robot Connected!" << std::endl;
	}
	else
	{
		return 0;
	};
	
	// ��ȡ�ؽ�λ������
	csvReader csv_reader;
	conciseRobotController elfin;

	string filePath = "./data/servoj_data.csv";
	int columnCounts = 6;
	EcRealVectorVector acs;
	
	// ��ȡ��Ӧ��Ϣ
	if (!csv_reader.fileDataRead(filePath, columnCounts, acs))
	{
		return false;
	};

	// ���Դ�ӡ�ļ���Ϣ
	for (int i = 0; i < 1; i++)
	{
		for (int j = 0; j < columnCounts; j++)
		{
			// �����ӡ��Ϣ��Ȼ���ӡ�س�
			cout << acs.at(i).at(j) << ", ";
		}
		std::cout << endl;
	}
	 
	
	EcRealVector initJoint = acs[0];

	EcRealVectorVector curJointV;

	elfin.moveJ(initJoint, 360, 50);

	// ��ʼ�ŷ�����
	double indServoTime = 0.015;
	double indLookaheadTime = 0.05;
	//servoTime = 0.025   # �����ŷ�����Ϊ 25ms��������С������ 15ms
	//	# ����ǰհʱ�䣬ǰհʱ��Խ�󣬹켣Խƽ������Խ�ͺ�
	//	lookaheadTime = 0.2 # ����ǰհʱ��Ϊ 200ms, ������ 0.05s~0.2s ֮��
	elfin.start_servo(indServoTime, indLookaheadTime);


	std::vector<double> Time_v;
	double begin; // ���ȳ߿�ʼʱ��
	begin = clock();

	for (int i = 0; i < acs.size(); i++)
	{
		elfin.servoj(acs[i]);
		Time_v.push_back((clock() - begin) / 1000.); // ��¼�·�ʱ��ʱ��

		while (1)
		{
			if ((clock() - begin) >= ((i + 1) * indServoTime * 1000))
			{
				break;
			}
			Sleep(0.001);
		}
		// ��ȡ�����˹ؽ�λ��
		EcRealVector curPos;
		elfin.readAcsPos(curPos);
		curJointV.push_back(curPos);
	}

	HRIF_DisConnect(0);

	// д�ļ�
	std::string cmdpath = "./data/cmd.csv";
	csv_reader.writeJoints(cmdpath, acs);

	std::string exepath = "./data/execute.csv";
	csv_reader.writeJoints(exepath, curJointV);

	// ��¼ʱ��
	std::string timepath = "./data/time.csv";
	csv_reader.writeTime(timepath, Time_v);

	return true;
}