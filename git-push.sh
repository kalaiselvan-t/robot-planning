branch_name=$(git symbolic-ref -q HEAD)
branch_name=${branch_name##refs/heads/}
git add .
git commit -m "$1"
git push github $branch_name
echo "pushed to $branch_name"
