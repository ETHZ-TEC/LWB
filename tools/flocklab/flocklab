#!/bin/bash
#
# $Id: flocklab 2489 2013-11-06 12:46:39Z walserc $
#
# command line tool for Flocklab
# usage:
#
# -v <testconfig.xml>: validate 
# -c <testconfig.xml>: create
# -a <testid>:         abort
# -d <testid>:         delete

SERVER_URL=https://www.flocklab.ethz.ch/user
CURL_PARAMS=-s
if [ -f .flocklabauth ]
then
  source ./.flocklabauth
else
  if [ -f $HOME/.flocklabauth ]
  then
    source $HOME/.flocklabauth
  fi
fi

usage()
{
cat << EOF
command line tool for flocklab

usage: $0 options

OPTIONS:
   -v <testconfig.xml>: validate 
   -c <testconfig.xml>: create
   -a <testid>:         abort
   -d <testid>:         delete
EOF
}

password()
{
cat << EOF
Wrong username / password
please check your settings in .flocklabauth
EOF
}


while getopts “v:c:a:d:” OPTION
do
  case $OPTION in
    v)
      RESPONSE=`curl $CURL_PARAMS -o - -F username="$USER" -F password="$PASSWORD" -F first="no" -F xmlfile=@$OPTARG $SERVER_URL/xmlvalidate.php`
      if [ $? -eq 0 ]
      then
        if [ `echo $RESPONSE | wc -m` -lt 2 ]
        then
          password
          exit
        fi
        SUCCESS=`echo $RESPONSE | sed '/<p>The file validated correctly.<\/p>/!d' | wc -l`
        if [ $SUCCESS -eq 0 ]
        then
          echo $RESPONSE | sed 's/.*<!-- cmd -->\(.*\)<!-- cmd -->.*/\1/;s/<li>/\n/g;s/<[^>]*>//g'
        else
          echo 'The file validated correctly.'
        fi
      else
        echo 'There were connection problems contacting the FlockLab server '$SERVER_URL
      fi
      ;;
    c)
      RESPONSE=`curl $CURL_PARAMS -o - -F username="$USER" -F password="$PASSWORD" -F first="no" -F xmlfile=@$OPTARG $SERVER_URL/newtest.php`
      if [ $? -eq 0 ]
      then
        if [ `echo $RESPONSE | wc -m` -lt 2 ]
        then
          password
          exit
        fi
        SUCCESS=`echo $RESPONSE | sed '/<!-- cmd --><p>Test (Id [0-9]*) successfully added.<\/p>/!d' | wc -l`
        if [ $SUCCESS -eq 0 ]
        then
          echo $RESPONSE | sed 's/.*<!-- cmd -->\(.*\)<!-- cmd -->.*/\1/;s/<li>/\n/g;s/<[^>]*>//g'
        else
          echo $RESPONSE | sed 's/.*<!-- flocklabscript\,\([0-9]*\),\([0-9]\{4\}-[0-9]\{2\}-[0-9]\{2\}T[0-9]\{2\}:[0-9]\{2\}:[0-9]\{2\}+[0-9]\{4\}\),\([0-9]*\)-->.*/Test successfully added. Test ID: \1, Starttime: \2 (\3)/'
        fi
      else
        echo 'There were connection problems contacting the FlockLab server '$SERVER_URL
      fi
      ;;
    a)
      RESPONSE=`curl $CURL_PARAMS -o - -F username="$USER" -F password="$PASSWORD" -F removeit="Remove test" -F testid=$OPTARG $SERVER_URL/test_abort.php`
      if [ $? -eq 0 ]
      then
        if [ `echo $RESPONSE | wc -m` -lt 2 ]
        then
          password
          exit
        fi
        SUCCESS=`echo $RESPONSE | sed '/<!-- cmd --><p>The test has been aborted.<\/p><!-- cmd -->/!d' | wc -l`
        if [ $SUCCESS -eq 0 ]
        then
          echo $RESPONSE | sed 's/.*<!-- cmd -->\(.*\)<!-- cmd -->.*/\1/;s/<li>/\n/g;s/<[^>]*>//g'
        else
          echo $RESPONSE | sed 's/.*<!-- cmd --><p>\(The test has been aborted\.\)<\/p><!-- cmd -->.*/\1/'
        fi
      else
        echo 'There were connection problems contacting the FlockLab server '$SERVER_URL
      fi
      ;;
     d)
      RESPONSE=`curl $CURL_PARAMS -o - -F username="$USER" -F password="$PASSWORD" -F removeit="Remove test" -F testid=$OPTARG $SERVER_URL/test_delete.php`
      if [ $? -eq 0 ]
      then
        if [ `echo $RESPONSE | wc -m` -lt 2 ]
        then
          password
          exit
        fi
        SUCCESS=`echo $RESPONSE | sed '/<!-- cmd --><p>The test has been removed.<\/p><!-- cmd -->/!d' | wc -l`
        if [ $SUCCESS -eq 0 ]
        then
          echo $RESPONSE | sed 's/.*<!-- cmd -->\(.*\)<!-- cmd -->.*/\1/;s/<li>/\n/g;s/<[^>]*>//g'
        else
          echo $RESPONSE | sed 's/.*<!-- cmd --><p>\(The test has been removed\.\)<\/p><!-- cmd -->.*/\1/'
        fi
      else
        echo 'There were connection problems contacting the FlockLab server '$SERVER_URL
      fi
      ;;
    ?)
      usage
      exit
      ;;
  esac
done

