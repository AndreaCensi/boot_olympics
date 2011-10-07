
options="$options --with-id"
# options="$options --trim-errors"
# options="$options --with-coverage"

nosetests $options  $*
