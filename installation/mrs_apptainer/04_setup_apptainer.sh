
## | ------------- add acados sourcing to .bashrc ------------- |

line='export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$HOME/git/acados/lib"'
num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .bashrc"

  # set exports to .bashrc
  echo "
$line" >> ~/.bashrc
fi

line='export ACADOS_SOURCE_DIR="$HOME/git/acados"'
num=`cat ~/.bashrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .bashrc"

  # set exports to .bashrc
  echo "
$line" >> ~/.bashrc
fi

## | ------------- add acados sourcing to .zshrc  ------------- |

line='export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$HOME/git/acados/lib"'
num=`cat ~/.zshrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .zshrc"

  # set exports to .bashrc
  echo "
$line" >> ~/.zshrc
fi

line='export ACADOS_SOURCE_DIR="$HOME/git/acados"'
num=`cat ~/.zshrc | grep "$line" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "Adding '$line' to your .zshrc"

  # set exports to .bashrc
  echo "
$line" >> ~/.zshrc
fi
