TO_UPLOAD="$1"

# zip the directory
zip -qr - $TO_UPLOAD | pv -bep -s $(du -bs $TO_UPLOAD | awk '{print $1}') > $TO_UPLOAD.zip

scp $TO_UPLOAD.zip zhange@bridges2.psc.edu:/ocean/projects/cis220074p/zhange/logs