import os, re
from   conan.tools.scm   import Git
from   conan.tools.files import replace_in_file

class conanPackages:
    def __parse (self, vPackage : str):
        packageComponent = (re.split('[/]', vPackage))
        return {'name' : packageComponent [0], 'version' : packageComponent [1] }
    
    def __createRepo (self, vRepoPath):
        print ('createRepoFolder')
        if not os.path.isdir (vRepoPath):
            os.mkdir (vRepoPath)
        os.chdir (vRepoPath)

    def __createFolderDownload (self, vPath : str):
        print ('createFolderDownload')
        if not os.path.isdir (vPath):
            os.mkdir (vPath)
        os.chdir (vPath)

    def __createPackage (self, vName : str, vVersion : str):
        print ('createPackage')
        self.run ('conan create . --name ' + vName + ' --version ' + vVersion)

    def __updateCMakeLists (self, vProjectPath, vSearch, vReplace):
        replace_in_file (self, os.path.join (vProjectPath, "CMakeLists.txt"), vSearch, vReplace, False)
    
    def __cloneRepo (self, vName : str, vVersion : str, vRepoPath : str, vRepoUrl : str):
        print ('cloneRepo')
        repoUrl = vRepoUrl + '/' + vName + '.git'
        print ('url', repoUrl)

        packagePath = vRepoPath + '/' + vName
        print (packagePath)
        if not os.path.isdir (packagePath):
            git = Git (self)
            git.clone    (url = repoUrl, target = vName)
            git.folder = vName 
            git.checkout (commit = 'tags/' + vVersion)

        os.chdir (packagePath + '/Conan')

    def install (self, vDownloadPath : str, vRepoPath : str, vRepoUrl : str, vPackages : str):
        print ('install')

        projectPath = os.getcwd ().replace ('/Build/Release','')
        conanPackages.__updateCMakeLists     (self, projectPath, "SET (PackageIncludePath )", "SET (PackageIncludePath " + self.packagePath + ")")
        conanPackages.__createFolderDownload (self, vDownloadPath)

        packageNames = []
        for package in vPackages:
            packageComponent = conanPackages.__parse (self, package)
            conanPackages.__createRepo               (self, vRepoPath)
            conanPackages.__cloneRepo                (self, packageComponent ['name'], packageComponent ['version'], vRepoPath, vRepoUrl)
            conanPackages.__createPackage            (self, packageComponent ['name'], packageComponent ['version'])

            packageNames.append (packageComponent ['name'])
        conanPackages.__updateCMakeLists (self, projectPath, "SET (PackageNames )", "SET (PackageNames " + " ".join (packageNames) + ")")
