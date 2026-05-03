
$ZIP_FILE = 'pcl-superbuild-x86.zip'
if [ -e $ZIP_FILE ]; then
    # ë∂ç›Ç∑ÇÈèÍçá
    echo "File $CFG_FILE exists."
else
    # ë∂ç›ÇµÇ»Ç¢èÍçá
    wget https://ci.appveyor.com/api/buildjobs/snk6yvrh7m37y38d/artifacts/$ZIP_FILE
fi

mkdir -p aar/pclmobile/libs/x86
cd aar/pclmobile/libs/x86
unzip -o ../../../pcl-superbuild-x86.zip
cd ../../../


$ZIP_FILE = 'pcl-superbuild-x86_64.zip'
if [ -e $ZIP_FILE ]; then
    # ë∂ç›Ç∑ÇÈèÍçá
    echo "File $CFG_FILE exists."
else
    # ë∂ç›ÇµÇ»Ç¢èÍçá
    wget https://ci.appveyor.com/api/buildjobs/hjwxdo3rh2yo04cc/artifacts/$ZIP_FILE
fi

mkdir -p aar/pclmobile/libs/x86_64
cd aar/pclmobile/libs/x86_64
unzip -o ../../../pcl-superbuild-x86_64.zip
cd ../../../


ZIP_FILE = ='pcl-superbuild-arm64-v8a.zip'
if [ -e ZIP_FILE ]; then
    # ë∂ç›Ç∑ÇÈèÍçá
    echo "File $CFG_FILE exists."
else
    # ë∂ç›ÇµÇ»Ç¢èÍçá
    wget https://ci.appveyor.com/api/buildjobs/9hyk1jk1420owgpg/artifacts/$ZIP_FILE
fi

mkdir -p aar/pclmobile/libs/arm64-v8a
cd aar/pclmobile/libs/arm64-v8a
unzip -o ../../../pcl-superbuild-arm64-v8a.zip
cd ../../../


ZIP_FILE = ='pcl-superbuild-armeabi.zip'
if [ -e $ZIP_FILE ]; then
    # ë∂ç›Ç∑ÇÈèÍçá
    echo "File $ZIP_FILE exists."
else
    # ë∂ç›ÇµÇ»Ç¢èÍçá
    wget https://ci.appveyor.com/api/buildjobs/fpulk58mq791aa24/artifacts/$ZIP_FILE
fi

mkdir -p aar/pclmobile/libs/armeabi

cd aar/pclmobile/libs/armeabi
unzip -o ../../../pcl-superbuild-armeabi.zip
cd ../../../


ZIP_FILE = ='pcl-superbuild-armeabi-v7a.zip'
if [ -e $ZIP_FILE ]; then
    # ë∂ç›Ç∑ÇÈèÍçá
    echo "File $ZIP_FILE exists."
else
    # ë∂ç›ÇµÇ»Ç¢èÍçá
    wget https://ci.appveyor.com/api/buildjobs/bg74r208w79oebyj/artifacts/$ZIP_FILE
fi

mkdir -p aar/pclmobile/libs/armeabi-v7a

cd aar/pclmobile/libs/armeabi-v7a
unzip -o ../../../pcl-superbuild-armeabi-v7a.zip
cd ../../../
