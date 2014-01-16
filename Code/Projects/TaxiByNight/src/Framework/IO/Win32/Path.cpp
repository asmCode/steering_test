#ifdef _WIN32

#include <Windows.h>

#include "../Path.h"

bool Path::IsFileExists(const char *fileName)
{
	if (GetFileAttributesA(fileName) != INVALID_FILE_ATTRIBUTES)
		return true;
	else
		return false;
}

void Path::GetAllFiles(std::vector<std::string> &files,
					   const std::string &path,
					   const std::string &filter)
{
	WIN32_FIND_DATAA fd;

	HANDLE hfile = FindFirstFileA((path + filter).c_str(), &fd);
	if (hfile != INVALID_HANDLE_VALUE)
	{
		BOOL doSearch = true;
		while (doSearch)
		{
			files.push_back(fd.cFileName);
			doSearch = FindNextFileA(hfile, &fd);
		}

		FindClose(hfile);
	}
}

#else

#include "../Path.h"
#include <Utils/StringUtils.h>
#include <FIo.h>

using namespace Tizen::Io;

bool Path::IsFileExists(const char *fileName)
{
	return File::IsFileExist(fileName);
}

void Path::GetAllFiles(std::vector<std::string> &files,
					   const std::string &path,
					   const std::string &filter)
{
	Directory* pDir = new Directory();
	DirEnumerator* pDirEnum;

	result r = pDir->Construct(path.c_str());
	if (r != E_SUCCESS)
	{
		AppLog(GetErrorMessage(r));
		delete pDir;
		return;
	}

    pDirEnum = pDir->ReadN();

	while (pDirEnum->MoveNext() == E_SUCCESS)
	{
		DirEntry entry = pDirEnum->GetCurrentDirEntry();

		if (entry.IsDirectory())
			continue;

		Path fileName(StringUtils::ToNarrow(entry.GetName().GetPointer()));

		if (fileName.GetExt() == filter.substr(filter.size() - 3))
		{
			files.push_back(fileName.GetFilenameExt());
		}
	}

	delete pDir;
	delete pDirEnum;
}

#endif
