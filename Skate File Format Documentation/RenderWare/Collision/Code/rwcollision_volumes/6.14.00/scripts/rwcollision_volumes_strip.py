import SimpleCodeStripper

codeStripper = SimpleCodeStripper.Strip(
        folderComponentsToExcludeFromCopy=['build', 'tests'],
        )

srcFolders = codeStripper.GenerateRecursiveFolderList(['source'])
srcFoldersToSkip = codeStripper.MatchingFolders(srcFolders, codeStripper.GetDefaultExcludedFolders() )
xmlSrcFiles = codeStripper.GenerateFilteredFileList(srcFolders - srcFoldersToSkip , ['.xml'])
codeStripper.CopyFiles(xmlSrcFiles)
