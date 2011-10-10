
options="$options --with-id"
# options="$options --trim-errors"
# options="$options --with-coverage"
options="--with-coverage --cover-html --cover-html-dir coverage_information --cover-package=bootstrapping_olympics"

nosetests $options  $*
